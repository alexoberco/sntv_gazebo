// sntv::StreamSystem — a Gazebo Sim system that renders a camera rig, composes it and
// encodes it with NVENC without the image ever leaving the GPU, and serves it over SNTV.
//
// Copyright 2026 Alejandro Bermúdez Fajardo (alexoberco)
// SPDX-License-Identifier: Apache-2.0
//
// Why: a Gazebo camera sensor reads every image back from the GPU synchronously, strips the
// alpha channel pixel by pixel, copies it several times and ships it over gz-transport while
// physics waits. Measured on an RTX 5060 Ti: a 150-degree wide-angle sensor reached 45 Hz at
// 1600 px, 37 Hz at 1920 and 25 Hz at 2160, with the GPU at 6-7 %, and it ignores
// <anti_aliasing>. This plugin does the same job at 2880 px per eye, 45-60 fps, real-time
// factor 1.0.
//
// What happens every frame, inside Gazebo's render thread:
//   1. Up to 5 ordinary cameras per view (cube faces: front, left, right, up, down) render
//      with MSAA. A narrow lens needs only the front one, with its field of view set to fit.
//   2. A private OpenGL context, shared with Ogre's (so Ogre's cached state is never touched),
//      composes each view with a fragment shader (perspective, equidistant fisheye or sine
//      fisheye) with 4 samples per pixel, views side by side.
//   3. NVENC encodes that texture directly (OpenGL input: no CUDA, no copy to the CPU).
//   4. A network thread serves the access units over SNTV (see the sntv repo), each with its
//      simulation timestamp and the world pose every view was rendered from.
//
// Where it hooks in: an Ogre::FrameListener. Scene::PostRender in gz-rendering (ogre2) fires
// frameRenderingQueued on the render thread after the sensors have been drawn and before the
// frame's data is released, with the scene graph at this frame's simulation time. The render
// thread only runs when some sensor is due, so a tiny camera sensor (the "clock sensor",
// 16x16 px, which this plugin subscribes to so Gazebo keeps it alive) makes it run at the
// stream's frame rate. That sensor is also the rig's anchor: its pose is the rig centre and
// its forward axis is the optical axis. Ruled out, measured or read in the source:
//  - gz-sim's PreRender/Render events: with a single subscriber the Sensors system renders on
//    EVERY physics step (1 kHz; Sensors.cc, hasRenderConnections) and slows the simulation;
//  - ConnectNewImageFrame on the clock camera: plain gz-rendering cameras do not emit it;
//  - the clock topic callback: gz-transport delivers it on another thread, without a GL context.
//
// Requires Gazebo started with --headless-rendering (EGL). With GLX there is no EGL context to
// share, and the plugin disables itself with an error.

#define GL_GLEXT_PROTOTYPES 1
#include <GL/glcorearb.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>

#include <arpa/inet.h>
#include <dlfcn.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <ffnvcodec/nvEncodeAPI.h>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/int64.pb.h>
#include <gz/plugin/Register.hh>
#include <OgreFrameListener.h>
#include <OgreRoot.h>

#include <gz/rendering/Camera.hh>
#include <gz/rendering/Light.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/sim/System.hh>
#include <gz/sim/rendering/Events.hh>
#include <gz/transport/Node.hh>

#include "sntv/sntv.h"

// To stderr rather than gzmsg: ros_gz_sim starts gz sim at verbosity 1, where gzmsg is silent,
// and these lines (startup and the 5 s statistics) are the ones worth reading.
#define SNTVLOG std::cerr << "[sntv] "

namespace sntv
{
namespace gzs = gz::sim;
namespace gzr = gz::rendering;
using Clock = std::chrono::steady_clock;

static double ms_since(Clock::time_point t0)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

constexpr double kPi = 3.14159265358979323846;
constexpr int kMaxViews = 2;
constexpr int kFaces = 5;

// ------------------------------------------------------------------------------ shaders
static const char *kVs = R"(#version 450 core
void main() {
  vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
  gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
})";

// Image coordinates: gl_FragCoord.y = 0 is texture row 0, which NVENC takes as the TOP row.
// x to the right, y downwards. Optical frame: X forward, Y left, Z up.
static const char *kFs = R"(#version 450 core
layout(location = 0) out vec4 color;
uniform sampler2D face[5];
uniform vec3 fwd[5];        // each face's axis in the view frame
uniform vec3 rgt[5];        // right of the face image
uniform vec3 upv[5];        // up of the face image
uniform float faceTan[5];   // tan(half field of view) of each face
uniform int nFaces;
uniform float viewSize;     // px per side of a view
uniform float x0;           // first column of this view
uniform int projection;     // 0 perspective, 1 equidistant, 2 sine (SNTV numbering)
uniform float halfFov;      // rad at r = 0.5
uniform float lensC2;
uniform float cutoff;       // rad: black beyond
uniform int toSrgb;         // faces are sRGB: what is read is linear, encode back

vec3 sampleDir(vec3 d) {
  int best = 0; float bx = dot(d, fwd[0]);
  for (int i = 1; i < nFaces; ++i) { float x = dot(d, fwd[i]); if (x > bx) { bx = x; best = i; } }
  if (bx <= 0.0) return vec3(0.0);
  float k = 1.0 / (bx * faceTan[best]);
  vec2 t = vec2(0.5 + 0.5 * dot(d, rgt[best]) * k, 0.5 - 0.5 * dot(d, upv[best]) * k);
  if (any(lessThan(t, vec2(0.0))) || any(greaterThan(t, vec2(1.0)))) return vec3(0.0);
  // Constant indices: sampler arrays cannot be indexed by a variable in GLSL 4.5.
  if (best == 0) return texture(face[0], t).rgb;
  if (best == 1) return texture(face[1], t).rgb;
  if (best == 2) return texture(face[2], t).rgb;
  if (best == 3) return texture(face[3], t).rgb;
  return texture(face[4], t).rgb;
}

vec3 shade(vec2 px) {
  vec2 p = (px - vec2(x0, 0.0)) / viewSize;    // 0..1, y downwards
  vec2 d = p - 0.5;
  if (projection == 0) {
    float t = tan(halfFov);
    return sampleDir(normalize(vec3(1.0, -2.0 * d.x * t, -2.0 * d.y * t)));
  }
  float r = length(d);
  float theta;
  if (projection == 2) {
    float sv = (r / 0.5) * sin(halfFov / lensC2);
    if (sv >= 1.0) return vec3(0.0);
    theta = lensC2 * asin(sv);
  } else {
    theta = r / 0.5 * halfFov;
  }
  if (theta > cutoff) return vec3(0.0);
  vec2 n = r > 1e-6 ? d / r : vec2(0.0);
  return sampleDir(vec3(cos(theta), -sin(theta) * n.x, -sin(theta) * n.y));
}

vec3 lin2srgb(vec3 c) {
  return mix(c * 12.92, 1.055 * pow(c, vec3(1.0 / 2.4)) - 0.055, step(0.0031308, c));
}

void main() {
  vec2 px = gl_FragCoord.xy;
  // 4 samples on a rotated grid inside the pixel: smooths what the resampling from the faces
  // leaves jagged (MSAA only covers geometry edges).
  vec3 c = 0.25 * (shade(px + vec2(0.125, 0.375)) + shade(px + vec2(-0.375, 0.125))
                 + shade(px + vec2(0.375, -0.125)) + shade(px + vec2(-0.125, -0.375)));
  color = vec4(toSrgb == 1 ? lin2srgb(c) : c, 1.0);
})";

// ------------------------------------------------------------------------------ plugin
class StreamSystem;
struct FrameListener : public Ogre::FrameListener
{
  StreamSystem *owner{nullptr};
  bool frameRenderingQueued(const Ogre::FrameEvent &) override;
};

class StreamSystem : public gzs::System, public gzs::ISystemConfigure
{
public:
  ~StreamSystem() override
  {
    // GL, NVENC and the cameras were released by Teardown on the render thread. If Gazebo never
    // called it, they are abandoned on purpose: the scene and the context may already be gone.
    if (this->listenerAdded) {
      if (Ogre::Root *root = Ogre::Root::getSingletonPtr()) root->removeFrameListener(&this->listener);
    }
    for (auto &view : this->cams) for (auto &c : view) if (c) new gzr::CameraPtr(std::move(c));
    if (this->clockCam) new gzr::CameraPtr(std::move(this->clockCam));
    if (this->scene) new gzr::ScenePtr(std::move(this->scene));
    if (this->teardownConn) new gz::common::ConnectionPtr(std::move(this->teardownConn));
    this->run = false;
    this->queueCv.notify_all();
    this->encCv.notify_all();
    if (this->listenFd >= 0) { shutdown(this->listenFd, SHUT_RDWR); close(this->listenFd); }
    if (this->encThread.joinable()) this->encThread.join();
    if (this->netThread.joinable()) this->netThread.join();
  }

  void Configure(const gzs::Entity &, const std::shared_ptr<const sdf::Element> &_sdf,
                 gzs::EntityComponentManager &, gzs::EventManager &_events) override
  {
    auto sdf = _sdf->Clone();
    this->port = sdf->Get<int>("port", 8766).first;
    this->bindAddress = sdf->Get<std::string>("bind_address", "0.0.0.0").first;
    this->viewSize = sdf->Get<int>("view_size", 1920).first & ~1;   // 4:2:0 needs even sizes
    this->views = sdf->Get<std::string>("layout", "stereo").first == "mono" ? 1 : 2;
    this->ipd = sdf->Get<double>("ipd", 0.063).first;
    const std::string proj = sdf->Get<std::string>("projection", "equidistant").first;
    this->projection = proj == "perspective" ? SNTV_PROJ_PERSPECTIVE
                     : proj == "sine" ? SNTV_PROJ_FISHEYE_SINE : SNTV_PROJ_FISHEYE_EQUIDISTANT;
    this->fovDeg = sdf->Get<double>("fov_deg", this->projection == SNTV_PROJ_PERSPECTIVE ? 90.0 : 150.0).first;
    this->lensC2 = sdf->Get<double>("lens_c2", 1.0).first;
    this->cutoffDeg = std::min(sdf->Get<double>("cutoff_deg", 90.0).first, 90.0);
    this->faceScale = sdf->Get<double>("face_scale", 1.0).first;
    this->peripheralScale = std::clamp(sdf->Get<double>("peripheral_scale", 0.7).first, 0.25, 1.0);
    this->msaa = sdf->Get<int>("msaa", 4).first;
    this->nearClip = sdf->Get<double>("near", 0.02).first;
    this->farClip = sdf->Get<double>("far", 200.0).first;
    this->shadows = sdf->Get<bool>("shadows", true).first;
    this->codec = sdf->Get<std::string>("codec", "hevc").first == "h264" ? SNTV_CODEC_H264 : SNTV_CODEC_HEVC;
    this->bitrate = sdf->Get<int>("bitrate", 40000000).first;
    this->fps = std::max(1, sdf->Get<int>("fps", 45).first);
    this->sendPoses = sdf->Get<bool>("view_poses", true).first;
    this->clockName = sdf->Get<std::string>("clock_sensor", "sntv_clock").first;
    const auto clockTopic = sdf->Get<std::string>("clock_topic", "/sntv/clock").first;
    const auto frameTopic = sdf->Get<std::string>("frame_topic", "").first;

    if (this->projection == SNTV_PROJ_PERSPECTIVE) this->fovDeg = std::clamp(this->fovDeg, 1.0, 150.0);
    this->outW = this->views * this->viewSize;
    this->outH = this->viewSize;
    if (this->codec == SNTV_CODEC_H264 && this->outW > 4096) {
      SNTVLOG << "WARNING: " << this->outW << " px wide is beyond H.264's 4096 on NVENC; use codec hevc.\n";
    }
    this->PlanFaces();

    // The clock sensor only renders while someone listens (Sensors disables sensors without
    // subscribers): this empty subscription keeps it alive.
    this->node.Subscribe(clockTopic, &StreamSystem::OnClockImage, this);
    if (!frameTopic.empty()) {
      this->framePub = this->node.Advertise<gz::msgs::Int64>(frameTopic);
      this->publishFrames = true;
    }
    // RenderTeardown does not count as a "render connection" for Sensors (only PreRender, Render
    // and PostRender force renders), so it is safe to use for closing NVENC and GL on the render
    // thread. Without closing the NVENC session, the Gazebo process hung on exit.
    this->teardownConn = _events.Connect<gzs::events::RenderTeardown>(std::bind(&StreamSystem::Teardown, this));
    this->netThread = std::thread(&StreamSystem::NetLoop, this);

    static const char *projNames[] = {"perspective", "equidistant", "sine"};
    SNTVLOG << this->outW << "x" << this->outH << (this->codec == SNTV_CODEC_HEVC ? " HEVC" : " H.264")
            << " on TCP " << this->bindAddress << ":" << this->port << ", " << (this->views == 2 ? "stereo" : "mono")
            << ", " << projNames[this->projection] << " " << this->fovDeg << " deg, "
            << this->nFaces << (this->nFaces == 1 ? " face" : " faces") << " per view ("
            << this->faceSize << (this->nFaces > 1 ? "/" + std::to_string(this->peripheralSize) : std::string())
            << " px, MSAA " << this->msaa << "): " << this->centerPpd << " px/deg at the centre.\n";
  }

private:
  struct Slot;

  // Decides how many cube faces each view needs and how big they are, so that the faces offer
  // at their centre as many px/degree as the composed view does at its own.
  void PlanFaces()
  {
    const double halfFov = this->fovDeg * 0.5 * kPi / 180.0;
    const double cutoff = this->cutoffDeg * kPi / 180.0;
    // Angle of a ray at normalised radius r (centre 0, edge of the view 0.5).
    auto thetaAt = [&](double r) {
      if (this->projection == SNTV_PROJ_FISHEYE_SINE) {
        const double sv = std::min(1.0, r / 0.5 * std::sin(halfFov / this->lensC2));
        return std::min(cutoff, this->lensC2 * std::asin(sv));
      }
      return std::min(cutoff, r / 0.5 * halfFov);
    };
    double pxPerRad;   // at the centre of the composed view
    double needHalf;   // half field of view a single front face must have
    if (this->projection == SNTV_PROJ_PERSPECTIVE) {
      pxPerRad = this->viewSize * 0.5 / std::tan(halfFov);
      needHalf = halfFov;
    } else {
      pxPerRad = this->projection == SNTV_PROJ_FISHEYE_SINE
        ? this->viewSize * 0.5 / (this->lensC2 * std::sin(halfFov / this->lensC2))
        : this->viewSize * 0.5 / halfFov;
      // Along an axis the ray needs tan(theta) on that axis; on the diagonal it splits in two.
      needHalf = std::max(thetaAt(0.5), std::atan(std::tan(std::min(thetaAt(0.5 * std::sqrt(2.0)), 1.55)) / std::sqrt(2.0)));
    }
    this->centerPpd = pxPerRad * kPi / 180.0;
    // A single face is cheaper and sharper while its field of view stays moderate; past ~60
    // degrees per side its edges stretch, and five 90-degree faces do better.
    if (needHalf <= 60.0 * kPi / 180.0) {
      this->nFaces = 1;
      this->faceHalf[0] = needHalf;
      const double faceCentrePxPerRad = 0.5 / std::tan(needHalf);   // per pixel of face size
      this->faceSize = (static_cast<int>(std::ceil(pxPerRad / faceCentrePxPerRad * this->faceScale)) + 7) & ~7;
      this->peripheralSize = this->faceSize;
    } else {
      this->nFaces = kFaces;
      for (double &h : this->faceHalf) h = kPi / 4;
      this->faceSize = (static_cast<int>(std::ceil(2.0 * pxPerRad * this->faceScale)) + 7) & ~7;
      // The side faces are only used beyond ~45 degrees, where a fisheye asks far fewer px/degree:
      // rendering them full size took the GPU to 98 % at 2880 px per eye in the stereo case.
      this->peripheralSize = (static_cast<int>(std::ceil(this->faceSize * this->peripheralScale)) + 7) & ~7;
    }
  }

  // gz-transport thread. Only used to register the FrameListener as soon as the render engine
  // exists (the Sensors system loads it on its own thread).
  void OnClockImage(const gz::msgs::Image &)
  {
    if (this->listenerAdded) return;
    // Ogre::Root rather than Ogre2RenderEngine::Instance(): gz-rendering loads the engine as a
    // plugin and its gz singleton is not the one this .so sees. Ogre's lives in libOgreNextMain.
    Ogre::Root *root = Ogre::Root::getSingletonPtr();
    if (!root || !gzr::isEngineLoaded("ogre2")) return;
    this->listener.owner = this;
    // Ogre defers listener additions to the next frame, so registering from this thread does
    // not interfere with an iteration in progress on the render thread.
    root->addFrameListener(&this->listener);
    this->listenerAdded = true;
    SNTVLOG << "FrameListener registered with Ogre.\n";
  }

public:
  // Render thread, at the end of every Scene::PostRender.
  void OnRenderQueued()
  {
    if (this->failed || this->inFrame) return;
    if (!this->booted && !this->FindClock()) return;
    const auto t = this->scene->Time();
    // Only when a frame is due: the render thread also runs for other sensors.
    const auto period = std::chrono::nanoseconds(1000000000LL / this->fps);
    if (this->lastFrameTime.count() >= 0 && t - this->lastFrameTime < period * 9 / 10) return;
    this->lastFrameTime = t;
    this->inFrame = true;
    this->OnFrame();
    this->inFrame = false;
  }

private:
  // Render thread, before Gazebo destroys the scene and the engine.
  void Teardown()
  {
    if (this->listenerAdded) {
      if (Ogre::Root *root = Ogre::Root::getSingletonPtr()) root->removeFrameListener(&this->listener);
      this->listenerAdded = false;
    }
    this->failed = true;
    // Encoder thread first: it may be waiting on NVENC. It closes NVENC and its context.
    this->run = false;
    this->encCv.notify_all();
    if (this->encThread.joinable()) this->encThread.join();
    if (this->glReady && this->MakeMine()) {
      for (auto &slot : this->slots) {
        if (slot.fence) { glDeleteSync(slot.fence); slot.fence = nullptr; }
        glDeleteFramebuffers(1, &slot.fbo);
        glDeleteTextures(1, &slot.tex);
      }
      glDeleteProgram(this->prog);
      glDeleteVertexArrays(1, &this->vao);
      glDeleteSamplers(1, &this->sampler);
      this->MakeOgre();
      eglDestroyContext(this->dpy, this->myCtx);
      this->glReady = false;
    }
    for (auto &view : this->cams)
      for (auto &c : view)
        if (c && this->scene) { this->scene->DestroySensor(c); c.reset(); }
    this->clockCam.reset();
    this->scene.reset();
    SNTVLOG << "Closed: NVENC, GL and cameras released.\n";
  }

  // shadows=false turns shadows off in the whole scene. There is no way to drop them for these
  // cameras only: Ogre's shadow node belongs to the scene, and every camera renders the shadow
  // maps again (3 cascades of the directional light). Measured at 2880 px per eye, 10 faces:
  // 14.0 -> 8.8 ms of render-thread CPU. Rechecked every ~90 frames in case gz-sim reapplies it.
  void DisableShadows()
  {
    if (this->shadowCheck++ % 90 != 0) return;
    for (unsigned int i = 0; i < this->scene->LightCount(); ++i) {
      auto light = this->scene->LightByIndex(i);
      if (light && light->CastShadows()) {
        light->SetCastShadows(false);
        SNTVLOG << "Shadows off for light '" << light->Name() << "'.\n";
      }
    }
  }

  bool FindClock()
  {
    auto scene = gzr::sceneFromFirstRenderEngine();
    if (!scene) return false;
    for (unsigned int i = 0; i < scene->SensorCount(); ++i) {
      auto cam = std::dynamic_pointer_cast<gzr::Camera>(scene->SensorByIndex(i));
      if (!cam) continue;
      const std::string &n = cam->Name();
      if (n.size() >= this->clockName.size() &&
          n.compare(n.size() - this->clockName.size(), this->clockName.size(), this->clockName) == 0) {
        this->scene = scene;
        this->clockCam = cam;
        this->booted = true;
        SNTVLOG << "Clock sensor: " << n << "\n";
        return true;
      }
    }
    return false;
  }

  bool BuildRig()
  {
    auto parent = this->clockCam->Parent();
    if (!parent) { SNTVLOG << "ERROR: the clock sensor has no parent.\n"; return false; }
    const gz::math::Pose3d mount = this->clockCam->LocalPose();
    // Face rotation relative to the view. A gz camera looks along +X, up +Z, right -Y.
    const gz::math::Quaterniond rot[kFaces] = {
      gz::math::Quaterniond(0, 0, 0),             // front
      gz::math::Quaterniond(0, 0, kPi / 2),       // left
      gz::math::Quaterniond(0, 0, -kPi / 2),      // right
      gz::math::Quaterniond(0, -kPi / 2, 0),      // up
      gz::math::Quaterniond(0, kPi / 2, 0),       // down
    };
    for (int f = 0; f < kFaces; ++f) {
      this->axes[f][0] = rot[f] * gz::math::Vector3d::UnitX;
      this->axes[f][1] = rot[f] * -gz::math::Vector3d::UnitY;
      this->axes[f][2] = rot[f] * gz::math::Vector3d::UnitZ;
    }
    // View 0 is the left eye (+Y), view 1 the right one.
    for (int v = 0; v < this->views; ++v) {
      const double y = this->views == 1 ? 0.0 : (v == 0 ? 0.5 : -0.5) * this->ipd;
      this->viewOffset[v] = gz::math::Pose3d(0, y, 0, 0, 0, 0);
      for (int f = 0; f < this->nFaces; ++f) {
        auto cam = this->scene->CreateCamera(this->clockName + "_sntv_" + std::to_string(v) + "_" + std::to_string(f));
        const int side = f == 0 ? this->faceSize : this->peripheralSize;
        cam->SetImageWidth(side);
        cam->SetImageHeight(side);
        cam->SetImageFormat(gzr::PF_R8G8B8);
        cam->SetAspectRatio(1.0);
        cam->SetHFOV(gz::math::Angle(2.0 * this->faceHalf[f]));
        cam->SetAntiAliasing(this->msaa);
        cam->SetNearClipPlane(this->nearClip);
        cam->SetFarClipPlane(this->farClip);
        parent->AddChild(cam);
        cam->SetLocalPose(mount * this->viewOffset[v] * gz::math::Pose3d(gz::math::Vector3d::Zero, rot[f]));
        this->cams[v][f] = cam;
      }
    }
    return true;
  }

  bool InitGl()
  {
    this->dpy = eglGetCurrentDisplay();
    this->ogreCtx = eglGetCurrentContext();
    if (this->dpy == EGL_NO_DISPLAY || this->ogreCtx == EGL_NO_CONTEXT) {
      SNTVLOG << "ERROR: no EGL context on the render thread: start gz sim with --headless-rendering.\n";
      return false;
    }
    this->ogreDraw = eglGetCurrentSurface(EGL_DRAW);
    this->ogreRead = eglGetCurrentSurface(EGL_READ);
    EGLint cfgId = 0;
    EGLConfig cfg = nullptr;
    eglQueryContext(this->dpy, this->ogreCtx, EGL_CONFIG_ID, &cfgId);
    if (cfgId > 0) {
      const EGLint attrs[] = {EGL_CONFIG_ID, cfgId, EGL_NONE};
      EGLint n = 0;
      eglChooseConfig(this->dpy, attrs, &cfg, 1, &n);
      if (n < 1) cfg = nullptr;
    }
    eglBindAPI(EGL_OPENGL_API);
    this->eglCfg = cfg;
    const EGLint ctxAttrs[] = {EGL_CONTEXT_MAJOR_VERSION, 4, EGL_CONTEXT_MINOR_VERSION, 5,
                               EGL_CONTEXT_OPENGL_PROFILE_MASK, EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT, EGL_NONE};
    this->myCtx = eglCreateContext(this->dpy, cfg, this->ogreCtx, ctxAttrs);
    if (this->myCtx == EGL_NO_CONTEXT) {
      SNTVLOG << "ERROR: eglCreateContext failed (0x" << std::hex << eglGetError() << std::dec << ").\n";
      return false;
    }
    if (!this->MakeMine()) return false;

    auto compile = [](GLenum type, const char *src) {
      GLuint s = glCreateShader(type);
      glShaderSource(s, 1, &src, nullptr);
      glCompileShader(s);
      GLint ok = 0;
      glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
      if (!ok) {
        char log[2048];
        glGetShaderInfoLog(s, sizeof log, nullptr, log);
        SNTVLOG << "ERROR: shader: " << log << "\n";
      }
      return s;
    };
    this->prog = glCreateProgram();
    glAttachShader(this->prog, compile(GL_VERTEX_SHADER, kVs));
    glAttachShader(this->prog, compile(GL_FRAGMENT_SHADER, kFs));
    glLinkProgram(this->prog);
    GLint ok = 0;
    glGetProgramiv(this->prog, GL_LINK_STATUS, &ok);
    if (!ok) { SNTVLOG << "ERROR: the program does not link.\n"; this->MakeOgre(); return false; }
    glGenVertexArrays(1, &this->vao);
    // Own sampler: Ogre's textures may have a mipmapped filter and a single level, and would
    // then read as black (incomplete texture).
    glGenSamplers(1, &this->sampler);
    glSamplerParameteri(this->sampler, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glSamplerParameteri(this->sampler, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glSamplerParameteri(this->sampler, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glSamplerParameteri(this->sampler, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    for (auto &slot : this->slots) {
      glGenTextures(1, &slot.tex);
      glBindTexture(GL_TEXTURE_2D, slot.tex);
      glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA8, this->outW, this->outH);
      glGenFramebuffers(1, &slot.fbo);
      glBindFramebuffer(GL_DRAW_FRAMEBUFFER, slot.fbo);
      glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, slot.tex, 0);
      const GLenum st = glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER);
      if (st != GL_FRAMEBUFFER_COMPLETE) {
        SNTVLOG << "ERROR: incomplete FBO 0x" << std::hex << st << std::dec << "\n";
        this->MakeOgre();
        return false;
      }
    }
    // The textures must exist before the encoder's context registers them.
    glFinish();
    this->MakeOgre();
    SNTVLOG << "GL ready: " << glGetString(GL_RENDERER) << "\n";
    this->encThread = std::thread(&StreamSystem::EncodeLoop, this);
    return true;
  }

  bool MakeMine()
  {
    if (!eglMakeCurrent(this->dpy, this->ogreDraw, this->ogreRead, this->myCtx) &&
        !eglMakeCurrent(this->dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, this->myCtx)) {
      SNTVLOG << "ERROR: eglMakeCurrent (own context) failed (0x" << std::hex << eglGetError() << std::dec << ").\n";
      return false;
    }
    return true;
  }

  void MakeOgre() { eglMakeCurrent(this->dpy, this->ogreDraw, this->ogreRead, this->ogreCtx); }

  void OnFrame()
  {
    if (this->failed) return;
    if (!this->rigReady) {
      // New cameras get ready in the scene's PreRender of the next frame.
      this->rigReady = this->BuildRig();
      if (!this->rigReady) this->failed = true;
      return;
    }
    if (!this->glReady) {
      this->glReady = this->InitGl();
      if (!this->glReady) { this->failed = true; return; }
    }
    if (this->encFailed) { this->failed = true; return; }
    if (!this->encReady) return;   // NVENC starting on its thread
    if (!this->shadows) this->DisableShadows();
    const auto t0 = Clock::now();
    const int64_t stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(this->scene->Time()).count();

    GLuint tex[kMaxViews][kFaces] = {};
    for (int v = 0; v < this->views; ++v) {
      for (int f = 0; f < this->nFaces; ++f) {
        auto &cam = this->cams[v][f];
        cam->Render();
        cam->PostRender();
        cam->PrepareForExternalSampling();
        tex[v][f] = cam->RenderTextureGLId();
      }
    }
    // Ogre submits on its own context: a fence so ours waits for it on the GPU.
    GLsync rendered = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
    glFlush();
    const double renderMs = ms_since(t0);

    const auto t1 = Clock::now();
    if (!this->MakeMine()) { glDeleteSync(rendered); return; }
    glWaitSync(rendered, 0, GL_TIMEOUT_IGNORED);
    glDeleteSync(rendered);

    if (!this->texChecked) {
      GLint fmt = 0;
      glGetTextureLevelParameteriv(tex[0][0], 0, GL_TEXTURE_INTERNAL_FORMAT, &fmt);
      this->srgbFaces = (fmt == GL_SRGB8_ALPHA8 || fmt == GL_SRGB8);
      this->texChecked = true;
    }

    Slot &slot = this->slots[this->nextSlot];
    if (slot.state != Slot::kFree) {
      // The encoder is a full ring behind: skip this frame (the next one is an ordinary P frame,
      // since nothing encoded depends on this one).
      ++this->skipped;
      this->MakeOgre();
      return;
    }
    this->nextSlot = (this->nextSlot + 1) % kSlots;

    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, slot.fbo);
    glViewport(0, 0, this->outW, this->outH);
    glDisable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glDisable(GL_FRAMEBUFFER_SRGB);
    glUseProgram(this->prog);
    glBindVertexArray(this->vao);
    auto loc = [this](const char *n) { return glGetUniformLocation(this->prog, n); };
    glUniform1f(loc("viewSize"), static_cast<float>(this->viewSize));
    glUniform1f(loc("halfFov"), static_cast<float>(this->fovDeg * 0.5 * kPi / 180.0));
    glUniform1i(loc("projection"), static_cast<int>(this->projection));
    glUniform1f(loc("lensC2"), static_cast<float>(this->lensC2));
    glUniform1f(loc("cutoff"), static_cast<float>(this->cutoffDeg * kPi / 180.0));
    glUniform1i(loc("toSrgb"), this->srgbFaces ? 1 : 0);
    glUniform1i(loc("nFaces"), this->nFaces);
    for (int f = 0; f < kFaces; ++f) {
      const std::string i = "[" + std::to_string(f) + "]";
      const auto &a = this->axes[f];
      glUniform3f(loc(("fwd" + i).c_str()), a[0].X(), a[0].Y(), a[0].Z());
      glUniform3f(loc(("rgt" + i).c_str()), a[1].X(), a[1].Y(), a[1].Z());
      glUniform3f(loc(("upv" + i).c_str()), a[2].X(), a[2].Y(), a[2].Z());
      glUniform1f(loc(("faceTan" + i).c_str()), static_cast<float>(std::tan(this->faceHalf[f])));
      glUniform1i(loc(("face" + i).c_str()), f);
      glBindSampler(f, this->sampler);
    }
    glEnable(GL_SCISSOR_TEST);
    for (int v = 0; v < this->views; ++v) {
      for (int f = 0; f < this->nFaces; ++f) {
        glActiveTexture(GL_TEXTURE0 + f);
        glBindTexture(GL_TEXTURE_2D, tex[v][f]);
      }
      glUniform1f(loc("x0"), static_cast<float>(v * this->viewSize));
      glScissor(v * this->viewSize, 0, this->viewSize, this->viewSize);
      glDrawArrays(GL_TRIANGLES, 0, 3);
    }
    glDisable(GL_SCISSOR_TEST);
    for (int f = 0; f < kFaces; ++f) {
      glActiveTexture(GL_TEXTURE0 + f);
      glBindTexture(GL_TEXTURE_2D, 0);
      glBindSampler(f, 0);
    }
    glActiveTexture(GL_TEXTURE0);
    // No glFinish: waiting here for the GPU cost the render thread (and physics, which waits for
    // it) 2-6 ms per frame at 2880 px per eye. A fence, and the slot goes to the encoder thread.
    slot.fence = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
    glFlush();
    slot.stamp = stamp;
    // The pose each view was rendered from, from the scene graph at this very frame.
    slot.nPoses = 0;
    if (this->sendPoses) {
      const gz::math::Pose3d rig = this->clockCam->WorldPose();
      for (int v = 0; v < this->views; ++v) {
        const gz::math::Pose3d p = rig * this->viewOffset[v];
        const auto &q = p.Rot();
        slot.poses[v] = {static_cast<float>(p.Pos().X()), static_cast<float>(p.Pos().Y()),
                         static_cast<float>(p.Pos().Z()), static_cast<float>(q.W()), static_cast<float>(q.X()),
                         static_cast<float>(q.Y()), static_cast<float>(q.Z())};
      }
      slot.nPoses = this->views;
    }
    slot.state = Slot::kEncoding;
    {
      std::lock_guard<std::mutex> lock(this->encMutex);
      this->encQueue.push_back(&slot);
    }
    this->encCv.notify_one();
    const double composeMs = ms_since(t1);
    this->MakeOgre();

    if (this->publishFrames) {
      gz::msgs::Int64 fm;
      fm.set_data(stamp);
      this->framePub.Publish(fm);
    }
    this->Stats(renderMs, composeMs, ms_since(t0));
  }

  // ---------------------------------------------------------------- NVENC
  bool InitNvenc()
  {
    void *lib = dlopen("libnvidia-encode.so.1", RTLD_NOW);
    if (!lib) { SNTVLOG << "ERROR: libnvidia-encode.so.1 not found (NVIDIA driver).\n"; return false; }
    using CreateFn = NVENCSTATUS (*)(NV_ENCODE_API_FUNCTION_LIST *);
    auto create = reinterpret_cast<CreateFn>(dlsym(lib, "NvEncodeAPICreateInstance"));
    this->nv.version = NV_ENCODE_API_FUNCTION_LIST_VER;
    if (!create || create(&this->nv) != NV_ENC_SUCCESS) { SNTVLOG << "ERROR: NvEncodeAPICreateInstance failed.\n"; return false; }

    NV_ENC_OPEN_ENCODE_SESSION_EX_PARAMS open{};
    open.version = NV_ENC_OPEN_ENCODE_SESSION_EX_PARAMS_VER;
    open.deviceType = NV_ENC_DEVICE_TYPE_OPENGL;
    open.device = nullptr;
    open.apiVersion = NVENCAPI_VERSION;
    NVENCSTATUS s = this->nv.nvEncOpenEncodeSessionEx(&open, &this->enc);
    if (s != NV_ENC_SUCCESS) { SNTVLOG << "ERROR: NVENC OpenGL session failed (" << s << ").\n"; return false; }

    const GUID codecGuid = this->codec == SNTV_CODEC_HEVC ? NV_ENC_CODEC_HEVC_GUID : NV_ENC_CODEC_H264_GUID;
    NV_ENC_PRESET_CONFIG preset{};
    preset.version = NV_ENC_PRESET_CONFIG_VER;
    preset.presetCfg.version = NV_ENC_CONFIG_VER;
    s = this->nv.nvEncGetEncodePresetConfigEx(this->enc, codecGuid, NV_ENC_PRESET_P2_GUID,
                                              NV_ENC_TUNING_INFO_ULTRA_LOW_LATENCY, &preset);
    if (s != NV_ENC_SUCCESS) { SNTVLOG << "ERROR: preset failed (" << s << ").\n"; return false; }
    this->cfg = preset.presetCfg;
    this->cfg.gopLength = NVENC_INFINITE_GOPLENGTH;
    this->cfg.frameIntervalP = 1;   // no B frames
    this->cfg.rcParams.rateControlMode = NV_ENC_PARAMS_RC_CBR;
    this->cfg.rcParams.averageBitRate = this->bitrate;
    this->cfg.rcParams.maxBitRate = this->bitrate;
    // One frame of buffer: every frame comes out about the same size, no bursts to queue.
    this->cfg.rcParams.vbvBufferSize = this->bitrate / this->fps;
    this->cfg.rcParams.vbvInitialDelay = this->cfg.rcParams.vbvBufferSize;
    if (this->codec == SNTV_CODEC_HEVC) {
      this->cfg.encodeCodecConfig.hevcConfig.idrPeriod = NVENC_INFINITE_GOPLENGTH;
      this->cfg.encodeCodecConfig.hevcConfig.repeatSPSPPS = 1;
    } else {
      this->cfg.encodeCodecConfig.h264Config.idrPeriod = NVENC_INFINITE_GOPLENGTH;
      this->cfg.encodeCodecConfig.h264Config.repeatSPSPPS = 1;
    }

    NV_ENC_INITIALIZE_PARAMS init{};
    init.version = NV_ENC_INITIALIZE_PARAMS_VER;
    init.encodeGUID = codecGuid;
    init.presetGUID = NV_ENC_PRESET_P2_GUID;
    init.tuningInfo = NV_ENC_TUNING_INFO_ULTRA_LOW_LATENCY;
    init.encodeWidth = this->outW;
    init.encodeHeight = this->outH;
    init.darWidth = this->outW;
    init.darHeight = this->outH;
    init.frameRateNum = this->fps;
    init.frameRateDen = 1;
    init.enablePTD = 1;
    init.encodeConfig = &this->cfg;
    s = this->nv.nvEncInitializeEncoder(this->enc, &init);
    if (s != NV_ENC_SUCCESS) { SNTVLOG << "ERROR: nvEncInitializeEncoder failed (" << s << ").\n"; return false; }

    for (auto &slot : this->slots) {
      NV_ENC_INPUT_RESOURCE_OPENGL_TEX glTex{};
      glTex.texture = slot.tex;
      glTex.target = GL_TEXTURE_2D;
      NV_ENC_REGISTER_RESOURCE reg{};
      reg.version = NV_ENC_REGISTER_RESOURCE_VER;
      reg.resourceType = NV_ENC_INPUT_RESOURCE_TYPE_OPENGL_TEX;
      reg.width = this->outW;
      reg.height = this->outH;
      reg.pitch = this->outW * 4;
      reg.resourceToRegister = &glTex;
      reg.bufferFormat = NV_ENC_BUFFER_FORMAT_ABGR;   // bytes R, G, B, A in memory
      reg.bufferUsage = NV_ENC_INPUT_IMAGE;
      s = this->nv.nvEncRegisterResource(this->enc, &reg);
      if (s != NV_ENC_SUCCESS) { SNTVLOG << "ERROR: registering the texture failed (" << s << ").\n"; return false; }
      slot.reg = reg.registeredResource;

      NV_ENC_CREATE_BITSTREAM_BUFFER bs{};
      bs.version = NV_ENC_CREATE_BITSTREAM_BUFFER_VER;
      s = this->nv.nvEncCreateBitstreamBuffer(this->enc, &bs);
      if (s != NV_ENC_SUCCESS) { SNTVLOG << "ERROR: output buffer failed (" << s << ").\n"; return false; }
      slot.bs = bs.bitstreamBuffer;
    }
    SNTVLOG << "NVENC " << (this->codec == SNTV_CODEC_HEVC ? "HEVC " : "H.264 ") << this->outW << "x" << this->outH
            << ", P2 ultra low latency, " << this->bitrate / 1000000 << " Mbit/s CBR, OpenGL input.\n";
    return true;
  }

  // Own thread, with its own GL context (shared with the composer's): waits on each slot's
  // fence, encodes it and hands it to the collector. Neither the wait for the GPU nor NVENC
  // land on the render thread. NVENC is opened and closed here because its OpenGL session is
  // bound to the context current on the thread that uses it.
  void EncodeLoop()
  {
    const EGLint ctxAttrs[] = {EGL_CONTEXT_MAJOR_VERSION, 4, EGL_CONTEXT_MINOR_VERSION, 5,
                               EGL_CONTEXT_OPENGL_PROFILE_MASK, EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT, EGL_NONE};
    eglBindAPI(EGL_OPENGL_API);
    EGLContext ctx = eglCreateContext(this->dpy, this->eglCfg, this->myCtx, ctxAttrs);
    if (ctx == EGL_NO_CONTEXT || !eglMakeCurrent(this->dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, ctx)) {
      SNTVLOG << "ERROR: encoder context (0x" << std::hex << eglGetError() << std::dec << ").\n";
      this->encFailed = true;
      return;
    }
    if (!this->InitNvenc()) {
      this->encFailed = true;
      eglMakeCurrent(this->dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
      eglDestroyContext(this->dpy, ctx);
      return;
    }
    this->collectRun = true;
    this->collectThread = std::thread(&StreamSystem::CollectLoop, this);
    this->encReady = true;

    while (this->run) {
      Slot *slot = nullptr;
      {
        std::unique_lock<std::mutex> lock(this->encMutex);
        this->encCv.wait_for(lock, std::chrono::milliseconds(200),
                             [this] { return !this->encQueue.empty() || !this->run; });
        if (this->encQueue.empty()) continue;
        slot = this->encQueue.front();
        this->encQueue.pop_front();
      }
      const auto t0 = Clock::now();
      glClientWaitSync(slot->fence, GL_SYNC_FLUSH_COMMANDS_BIT, 200000000);
      glDeleteSync(slot->fence);
      slot->fence = nullptr;
      const double gpuMs = ms_since(t0);

      // Unmapping what the collector has finished needs this context.
      this->UnmapCollected();

      NV_ENC_MAP_INPUT_RESOURCE map{};
      map.version = NV_ENC_MAP_INPUT_RESOURCE_VER;
      map.registeredResource = slot->reg;
      bool queued = false;
      if (this->nv.nvEncMapInputResource(this->enc, &map) == NV_ENC_SUCCESS) {
        NV_ENC_PIC_PARAMS pic{};
        pic.version = NV_ENC_PIC_PARAMS_VER;
        pic.inputWidth = this->outW;
        pic.inputHeight = this->outH;
        pic.inputPitch = this->outW * 4;
        pic.inputBuffer = map.mappedResource;
        pic.bufferFmt = map.mappedBufferFmt;
        pic.outputBitstream = slot->bs;
        pic.pictureStruct = NV_ENC_PIC_STRUCT_FRAME;
        pic.inputTimeStamp = static_cast<uint64_t>(slot->stamp);
        if (this->forceIdr.exchange(false))
          pic.encodePicFlags = NV_ENC_PIC_FLAG_FORCEIDR | NV_ENC_PIC_FLAG_OUTPUT_SPSPPS;
        slot->mapped = map.mappedResource;
        slot->encodeStart = Clock::now();
        const NVENCSTATUS st = this->nv.nvEncEncodePicture(this->enc, &pic);
        if (st == NV_ENC_SUCCESS) {
          // To the collector: nvEncLockBitstream blocks until NVENC is done, and meanwhile the
          // next frame can start. Serially (wait GPU + NVENC) it was 18-19 ms per frame at 2880
          // px per eye, more than the 16.7 ms of 60 fps.
          {
            std::lock_guard<std::mutex> lock(this->collectMutex);
            this->collectQueue.push_back(slot);
          }
          this->collectCv.notify_one();
          queued = true;
        } else {
          SNTVLOG << "nvEncEncodePicture failed (" << st << ")\n";
          this->nv.nvEncUnmapInputResource(this->enc, map.mappedResource);
          slot->mapped = nullptr;
        }
      }
      this->gpuWaitMs = 0.9 * this->gpuWaitMs + 0.1 * gpuMs;
      if (!queued) slot->state = Slot::kFree;
    }

    this->collectRun = false;
    this->collectCv.notify_all();
    if (this->collectThread.joinable()) this->collectThread.join();
    this->UnmapCollected();
    for (auto &slot : this->slots) {
      if (slot.reg) this->nv.nvEncUnregisterResource(this->enc, slot.reg);
      if (slot.bs) this->nv.nvEncDestroyBitstreamBuffer(this->enc, slot.bs);
      slot.reg = nullptr;
      slot.bs = nullptr;
    }
    if (this->enc) this->nv.nvEncDestroyEncoder(this->enc);
    this->enc = nullptr;
    eglMakeCurrent(this->dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    eglDestroyContext(this->dpy, ctx);
  }

  // Collector (no GL context: locking the bitstream does not need one). Takes the access units
  // out in launch order and leaves the slot for the encoder thread to unmap.
  void CollectLoop()
  {
    while (this->collectRun) {
      Slot *slot = nullptr;
      {
        std::unique_lock<std::mutex> lock(this->collectMutex);
        this->collectCv.wait_for(lock, std::chrono::milliseconds(200),
                                 [this] { return !this->collectQueue.empty() || !this->collectRun; });
        if (this->collectQueue.empty()) continue;
        slot = this->collectQueue.front();
        this->collectQueue.pop_front();
      }
      NV_ENC_LOCK_BITSTREAM lock{};
      lock.version = NV_ENC_LOCK_BITSTREAM_VER;
      lock.outputBitstream = slot->bs;
      Packet pkt;
      if (this->nv.nvEncLockBitstream(this->enc, &lock) == NV_ENC_SUCCESS) {
        const auto *p = static_cast<const uint8_t *>(lock.bitstreamBufferPtr);
        pkt.au.assign(p, p + lock.bitstreamSizeInBytes);
        this->nv.nvEncUnlockBitstream(this->enc, slot->bs);
      }
      this->waitMs = 0.9 * this->waitMs + 0.1 * ms_since(slot->encodeStart);
      pkt.stamp = slot->stamp;
      if (slot->nPoses > 0) {
        pkt.meta.resize(4 + slot->nPoses * SNTV_VIEW_POSE_SIZE);
        sntv_write_view_poses(pkt.meta.data(), slot->poses.data(), static_cast<uint32_t>(slot->nPoses));
      }
      slot->state = Slot::kCollected;
      if (!pkt.au.empty()) this->Push(std::move(pkt));
    }
  }

  // Encoder thread, with its context current.
  void UnmapCollected()
  {
    for (auto &slot : this->slots) {
      if (slot.state == Slot::kCollected) {
        if (slot.mapped) this->nv.nvEncUnmapInputResource(this->enc, slot.mapped);
        slot.mapped = nullptr;
        slot.state = Slot::kFree;
      }
    }
  }

  // ---------------------------------------------------------------- network
  struct Packet
  {
    int64_t stamp{0};
    std::vector<uint8_t> meta;
    std::vector<uint8_t> au;
  };

  void Push(Packet &&pkt)
  {
    std::lock_guard<std::mutex> lock(this->queueMutex);
    if (!this->clientConnected) return;
    // A receiver that cannot keep up: drop the backlog and ask for an IDR, because a P frame
    // whose reference was dropped cannot be decoded.
    if (this->queue.size() >= 3) {
      this->queue.clear();
      this->forceIdr = true;
      ++this->dropped;
      return;
    }
    this->queue.push_back(std::move(pkt));
    this->queueCv.notify_one();
  }

  static bool SendAll(int fd, const void *buf, size_t n)
  {
    const auto *p = static_cast<const uint8_t *>(buf);
    while (n > 0) {
      const ssize_t r = send(fd, p, n, MSG_NOSIGNAL);
      if (r <= 0) return false;
      p += r;
      n -= static_cast<size_t>(r);
    }
    return true;
  }

  void NetLoop()
  {
    this->listenFd = socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    setsockopt(this->listenFd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(static_cast<uint16_t>(this->port));
    if (inet_pton(AF_INET, this->bindAddress.c_str(), &a.sin_addr) != 1) a.sin_addr.s_addr = INADDR_ANY;
    if (bind(this->listenFd, reinterpret_cast<sockaddr *>(&a), sizeof a) != 0 || listen(this->listenFd, 1) != 0) {
      SNTVLOG << "ERROR: cannot listen on " << this->bindAddress << ":" << this->port << "\n";
      return;
    }
    while (this->run) {
      sockaddr_in peer{};
      socklen_t len = sizeof peer;
      const int fd = accept(this->listenFd, reinterpret_cast<sockaddr *>(&peer), &len);
      if (fd < 0) continue;
      setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
      // A send that takes longer than 2 frames is a receiver that stopped reading: let it go.
      timeval tv{0, 2 * 1000000 / this->fps};
      setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
      char ip[32];
      inet_ntop(AF_INET, &peer.sin_addr, ip, sizeof ip);

      sntv_header h{};
      h.width = static_cast<uint32_t>(this->outW);
      h.height = static_cast<uint32_t>(this->outH);
      h.codec = static_cast<uint32_t>(this->codec);
      h.fps = static_cast<float>(this->fps);
      h.layout = this->views == 2 ? SNTV_LAYOUT_STEREO_SBS : SNTV_LAYOUT_MONO;
      h.projection = static_cast<uint32_t>(this->projection);
      h.fov_deg = static_cast<float>(this->fovDeg);
      h.lens_c2 = static_cast<float>(this->lensC2);
      h.flags = this->sendPoses ? SNTV_FLAG_VIEW_POSES : 0u;
      uint8_t hdr[SNTV_HEADER_SIZE];
      sntv_write_header(hdr, &h);
      if (!SendAll(fd, hdr, sizeof hdr)) { close(fd); continue; }
      {
        std::lock_guard<std::mutex> lock(this->queueMutex);
        this->queue.clear();
        this->clientConnected = true;
      }
      this->forceIdr = true;
      SNTVLOG << "Receiver connected from " << ip << "\n";
      while (this->run) {
        Packet item;
        {
          std::unique_lock<std::mutex> lock(this->queueMutex);
          this->queueCv.wait_for(lock, std::chrono::milliseconds(500),
                                 [this] { return !this->queue.empty() || !this->run; });
          if (this->queue.empty()) {
            // With nothing to send, a vanished receiver goes unnoticed: peek at the socket.
            char c;
            const ssize_t r = recv(fd, &c, 1, MSG_DONTWAIT | MSG_PEEK);
            if (r == 0) break;
            continue;
          }
          item = std::move(this->queue.front());
          this->queue.pop_front();
        }
        sntv_frame_header f{static_cast<uint32_t>(item.au.size()), static_cast<uint64_t>(item.stamp),
                            static_cast<uint32_t>(item.meta.size())};
        uint8_t fh[SNTV_FRAME_HEADER_SIZE];
        sntv_write_frame_header(fh, &f);
        if (!SendAll(fd, fh, sizeof fh) ||
            (!item.meta.empty() && !SendAll(fd, item.meta.data(), item.meta.size())) ||
            !SendAll(fd, item.au.data(), item.au.size()))
          break;
        this->bytesSent += item.au.size();
      }
      {
        std::lock_guard<std::mutex> lock(this->queueMutex);
        this->clientConnected = false;
        this->queue.clear();
      }
      close(fd);
      SNTVLOG << "Receiver disconnected.\n";
    }
  }

  void Stats(double render, double compose, double total)
  {
    ++this->nFrames;
    this->sRender += render;
    this->sCompose += compose;
    this->sTotal += total;
    this->maxTotal = std::max(this->maxTotal, total);
    const auto now = Clock::now();
    const double dt = std::chrono::duration<double>(now - this->lastStats).count();
    if (dt < 5.0) return;
    const double n = this->nFrames;
    SNTVLOG << n / dt << " fps | render faces " << this->sRender / n << " ms, compose " << this->sCompose / n
            << " | encoder: wait GPU " << this->gpuWaitMs << " ms, NVENC " << this->waitMs << " | total "
            << this->sTotal / n << " (worst " << this->maxTotal << ") | " << this->bytesSent * 8.0 / dt / 1e6
            << " Mbit/s" << (this->clientConnected ? "" : " (no receiver)") << ", dropped " << this->dropped
            << ", skipped " << this->skipped << "\n";
    this->skipped = 0;
    this->nFrames = 0;
    this->sRender = this->sCompose = this->sTotal = this->maxTotal = 0.0;
    this->bytesSent = 0;
    this->dropped = 0;
    this->lastStats = now;
  }

  // parameters
  int port{8766}, viewSize{1920}, views{2}, msaa{4}, bitrate{40000000}, fps{45};
  int faceSize{0}, peripheralSize{0}, nFaces{kFaces}, outW{0}, outH{0};
  uint32_t projection{SNTV_PROJ_FISHEYE_EQUIDISTANT}, codec{SNTV_CODEC_HEVC};
  double ipd{0.063}, fovDeg{150.0}, lensC2{1.0}, cutoffDeg{90.0}, faceScale{1.0}, peripheralScale{0.7};
  double nearClip{0.02}, farClip{200.0}, centerPpd{0.0};
  double faceHalf[kFaces]{};
  bool shadows{true}, sendPoses{true}, publishFrames{false};
  std::string bindAddress, clockName;
  unsigned int shadowCheck{0};

  // gz
  gz::transport::Node node;
  gz::transport::Node::Publisher framePub;
  bool booted{false}, inFrame{false};
  gz::common::ConnectionPtr teardownConn;
  std::atomic<bool> listenerAdded{false};
  FrameListener listener;
  std::chrono::steady_clock::duration lastFrameTime{-1};
  gzr::ScenePtr scene;
  gzr::CameraPtr clockCam;
  gzr::CameraPtr cams[kMaxViews][kFaces];
  gz::math::Pose3d viewOffset[kMaxViews];
  gz::math::Vector3d axes[kFaces][3];
  bool rigReady{false}, glReady{false}, failed{false}, texChecked{false}, srgbFaces{false};

  // GL
  EGLDisplay dpy{EGL_NO_DISPLAY};
  EGLContext ogreCtx{EGL_NO_CONTEXT}, myCtx{EGL_NO_CONTEXT};
  EGLSurface ogreDraw{EGL_NO_SURFACE}, ogreRead{EGL_NO_SURFACE};
  GLuint prog{0}, vao{0}, sampler{0};

  // Output ring: the render thread composes into one slot and launches NVENC while the encoder
  // thread waits for the previous one.
  struct Slot
  {
    static constexpr int kFree = 0, kEncoding = 1, kCollected = 2;
    Clock::time_point encodeStart{};
    GLuint tex{0}, fbo{0};
    GLsync fence{nullptr};
    NV_ENC_REGISTERED_PTR reg{nullptr};
    NV_ENC_OUTPUT_PTR bs{nullptr};
    NV_ENC_INPUT_PTR mapped{nullptr};
    int64_t stamp{0};
    std::array<sntv_view_pose, kMaxViews> poses{};
    int nPoses{0};
    std::atomic<int> state{kFree};
  };
  // 4: one being composed, one waiting on its fence, one in NVENC and one being collected.
  static constexpr int kSlots = 4;
  Slot slots[kSlots];
  int nextSlot{0};
  int skipped{0};
  std::thread encThread;
  std::mutex encMutex;
  std::condition_variable encCv;
  std::deque<Slot *> encQueue;
  double waitMs{0.0}, gpuWaitMs{0.0};
  EGLConfig eglCfg{nullptr};
  std::atomic<bool> encReady{false}, encFailed{false}, collectRun{false};
  std::thread collectThread;
  std::mutex collectMutex;
  std::condition_variable collectCv;
  std::deque<Slot *> collectQueue;

  // NVENC
  NV_ENCODE_API_FUNCTION_LIST nv{};
  void *enc{nullptr};
  NV_ENC_CONFIG cfg{};
  std::atomic<bool> forceIdr{true};

  // network
  std::atomic<bool> run{true};
  std::thread netThread;
  int listenFd{-1};
  std::mutex queueMutex;
  std::condition_variable queueCv;
  std::deque<Packet> queue;
  bool clientConnected{false};
  std::atomic<uint64_t> bytesSent{0};
  std::atomic<int> dropped{0};

  // statistics
  Clock::time_point lastStats{Clock::now()};
  int nFrames{0};
  double sRender{0}, sCompose{0}, sTotal{0}, maxTotal{0};
};

bool FrameListener::frameRenderingQueued(const Ogre::FrameEvent &)
{
  if (this->owner) this->owner->OnRenderQueued();
  return true;
}
}  // namespace sntv

GZ_ADD_PLUGIN(sntv::StreamSystem, gz::sim::System, sntv::StreamSystem::ISystemConfigure)

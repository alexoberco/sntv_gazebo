/*
 * SNTV wire protocol, version 4 (SNTV 1.0) — see SPEC.md.
 *
 * Header-only, C99, no dependencies. Encodes and decodes the stream header,
 * the frame header and the metadata records; reading and writing the socket is
 * up to the caller. Everything on the wire is little endian, so these helpers
 * also work on big-endian hosts.
 *
 * Copyright 2026 Alejandro Bermúdez Fajardo (alexoberco)
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SNTV_H
#define SNTV_H

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SNTV_VERSION 4u
#define SNTV_HEADER_SIZE 48u        /* v4.0; receivers must honour header_size */
#define SNTV_FRAME_HEADER_SIZE 20u  /* "FRAM" + au_size + timestamp + meta_size */
#define SNTV_MAX_AU_SIZE (64u * 1024u * 1024u)

enum sntv_codec { SNTV_CODEC_H264 = 1, SNTV_CODEC_HEVC = 2 };
enum sntv_layout { SNTV_LAYOUT_MONO = 0, SNTV_LAYOUT_STEREO_SBS = 1 };
enum sntv_projection {
  SNTV_PROJ_PERSPECTIVE = 0,
  SNTV_PROJ_FISHEYE_EQUIDISTANT = 1,
  SNTV_PROJ_FISHEYE_SINE = 2
};
enum sntv_flags { SNTV_FLAG_VIEW_POSES = 1u << 0 };
enum sntv_meta_type { SNTV_META_VIEW_POSES = 1 };

typedef struct {
  uint32_t version;
  uint32_t header_size;
  uint32_t width, height;
  uint32_t codec;       /* enum sntv_codec */
  float fps;            /* 0 = unknown */
  uint32_t layout;      /* enum sntv_layout */
  uint32_t projection;  /* enum sntv_projection */
  float fov_deg;        /* across one view's width */
  float lens_c2;
  uint32_t flags;       /* enum sntv_flags */
} sntv_header;

typedef struct {
  uint32_t au_size;
  uint64_t timestamp_ns;
  uint32_t meta_size;
} sntv_frame_header;

/* Pose of one view's optical frame (X forward, Y left, Z up) in the world. */
typedef struct {
  float px, py, pz;
  float qw, qx, qy, qz;
} sntv_view_pose;

#define SNTV_VIEW_POSE_SIZE 28u

/* ----------------------------------------------------------------- byte helpers */
static inline void sntv_put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline void sntv_put_u32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static inline void sntv_put_u64(uint8_t *p, uint64_t v)
{
  sntv_put_u32(p, (uint32_t)v);
  sntv_put_u32(p + 4, (uint32_t)(v >> 32));
}
static inline void sntv_put_f32(uint8_t *p, float f) { uint32_t v; memcpy(&v, &f, 4); sntv_put_u32(p, v); }
static inline uint16_t sntv_get_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t sntv_get_u32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline uint64_t sntv_get_u64(const uint8_t *p)
{
  return (uint64_t)sntv_get_u32(p) | ((uint64_t)sntv_get_u32(p + 4) << 32);
}
static inline float sntv_get_f32(const uint8_t *p) { uint32_t v = sntv_get_u32(p); float f; memcpy(&f, &v, 4); return f; }

static inline uint32_t sntv_view_count(const sntv_header *h)
{
  return h->layout == SNTV_LAYOUT_STEREO_SBS ? 2u : 1u;
}

/* ----------------------------------------------------------------- stream header */
/* Writes SNTV_HEADER_SIZE bytes. */
static inline void sntv_write_header(uint8_t out[SNTV_HEADER_SIZE], const sntv_header *h)
{
  memcpy(out, "SNTV", 4);
  sntv_put_u32(out + 4, SNTV_VERSION);
  sntv_put_u32(out + 8, SNTV_HEADER_SIZE);
  sntv_put_u32(out + 12, h->width);
  sntv_put_u32(out + 16, h->height);
  sntv_put_u32(out + 20, h->codec);
  sntv_put_f32(out + 24, h->fps);
  sntv_put_u32(out + 28, h->layout);
  sntv_put_u32(out + 32, h->projection);
  sntv_put_f32(out + 36, h->fov_deg);
  sntv_put_f32(out + 40, h->lens_c2);
  sntv_put_u32(out + 44, h->flags);
}

/*
 * Reading a header is two steps because its size depends on the version:
 *   1. read SNTV_HEADER_PREFIX bytes and call sntv_header_size() -> total size (0 = invalid);
 *   2. read the rest into the same buffer and call sntv_parse_header().
 * Versions 1-3 (pre-release, fixed size) are accepted and mapped onto v4 fields.
 */
#define SNTV_HEADER_PREFIX 12u

static inline uint32_t sntv_header_size(const uint8_t *p)
{
  if (memcmp(p, "SNTV", 4) != 0) return 0;
  const uint32_t version = sntv_get_u32(p + 4);
  switch (version) {
    case 1: return 20;
    case 2: return 32;
    case 3: return 36;
    default: break;
  }
  if (version < 4) return 0;
  /* v4 and later: header_size says it; anything below the v4 fields is malformed. */
  const uint32_t size = sntv_get_u32(p + 8);
  return size >= SNTV_HEADER_SIZE && size <= 4096 ? size : 0;
}

/* Returns 0 on success. 'p' holds sntv_header_size(p) bytes. */
static inline int sntv_parse_header(const uint8_t *p, sntv_header *h)
{
  const uint32_t size = sntv_header_size(p);
  if (!size) return -1;
  memset(h, 0, sizeof *h);
  h->version = sntv_get_u32(p + 4);
  h->header_size = size;
  h->lens_c2 = 1.0f;
  if (h->version <= 3) {
    /* Legacy: SNTV | version | width | height | codec [| lens | c2 | fov] [| fps].
     * Always a 150-degree fisheye stereo pair; v1 is equidistant. */
    h->width = sntv_get_u32(p + 8);
    h->height = sntv_get_u32(p + 12);
    h->codec = sntv_get_u32(p + 16);
    h->layout = SNTV_LAYOUT_STEREO_SBS;
    h->projection = SNTV_PROJ_FISHEYE_EQUIDISTANT;
    h->fov_deg = 150.0f;
    if (h->version >= 2) {
      h->projection = sntv_get_u32(p + 20) == 1 ? SNTV_PROJ_FISHEYE_SINE : SNTV_PROJ_FISHEYE_EQUIDISTANT;
      h->lens_c2 = sntv_get_f32(p + 24);
      h->fov_deg = sntv_get_f32(p + 28);
    }
    if (h->version >= 3) h->fps = sntv_get_f32(p + 32);
    return 0;
  }
  h->width = sntv_get_u32(p + 12);
  h->height = sntv_get_u32(p + 16);
  h->codec = sntv_get_u32(p + 20);
  h->fps = sntv_get_f32(p + 24);
  h->layout = sntv_get_u32(p + 28);
  h->projection = sntv_get_u32(p + 32);
  h->fov_deg = sntv_get_f32(p + 36);
  h->lens_c2 = sntv_get_f32(p + 40);
  h->flags = sntv_get_u32(p + 44);
  return 0;
}

/* ----------------------------------------------------------------- frame header */
/* Frame header size for a stream: legacy streams (v1-3) use 16 bytes and no metadata. */
static inline uint32_t sntv_frame_header_size(const sntv_header *h)
{
  return h->version <= 3 ? 16u : SNTV_FRAME_HEADER_SIZE;
}

static inline void sntv_write_frame_header(uint8_t out[SNTV_FRAME_HEADER_SIZE], const sntv_frame_header *f)
{
  memcpy(out, "FRAM", 4);
  sntv_put_u32(out + 4, f->au_size);
  sntv_put_u64(out + 8, f->timestamp_ns);
  sntv_put_u32(out + 16, f->meta_size);
}

/* Returns 0 on success. 'p' holds sntv_frame_header_size(h) bytes. */
static inline int sntv_parse_frame_header(const uint8_t *p, const sntv_header *h, sntv_frame_header *f)
{
  if (memcmp(p, "FRAM", 4) != 0) return -1;
  f->au_size = sntv_get_u32(p + 4);
  f->timestamp_ns = sntv_get_u64(p + 8);
  f->meta_size = h->version <= 3 ? 0u : sntv_get_u32(p + 16);
  if (f->au_size > SNTV_MAX_AU_SIZE || f->meta_size > 65536u) return -1;
  return 0;
}

/* ----------------------------------------------------------------- metadata */
/* Writes a view-pose record for 'n' views; returns the bytes written (4 + 28 n). */
static inline uint32_t sntv_write_view_poses(uint8_t *out, const sntv_view_pose *poses, uint32_t n)
{
  sntv_put_u16(out, SNTV_META_VIEW_POSES);
  sntv_put_u16(out + 2, (uint16_t)(n * SNTV_VIEW_POSE_SIZE));
  uint8_t *q = out + 4;
  for (uint32_t i = 0; i < n; ++i, q += SNTV_VIEW_POSE_SIZE) {
    const float v[7] = {poses[i].px, poses[i].py, poses[i].pz, poses[i].qw, poses[i].qx, poses[i].qy, poses[i].qz};
    for (int k = 0; k < 7; ++k) sntv_put_f32(q + 4 * k, v[k]);
  }
  return 4 + n * SNTV_VIEW_POSE_SIZE;
}

/* Finds the view-pose record in a metadata block. Returns the number of poses copied
 * into 'out' (at most 'max'), or 0 if the block has none. */
static inline uint32_t sntv_read_view_poses(const uint8_t *meta, uint32_t size, sntv_view_pose *out, uint32_t max)
{
  uint32_t off = 0;
  while (off + 4 <= size) {
    const uint16_t type = sntv_get_u16(meta + off), len = sntv_get_u16(meta + off + 2);
    if (off + 4 + len > size) return 0;
    if (type == SNTV_META_VIEW_POSES) {
      uint32_t n = len / SNTV_VIEW_POSE_SIZE;
      if (n > max) n = max;
      const uint8_t *q = meta + off + 4;
      for (uint32_t i = 0; i < n; ++i, q += SNTV_VIEW_POSE_SIZE) {
        out[i].px = sntv_get_f32(q);      out[i].py = sntv_get_f32(q + 4);  out[i].pz = sntv_get_f32(q + 8);
        out[i].qw = sntv_get_f32(q + 12); out[i].qx = sntv_get_f32(q + 16);
        out[i].qy = sntv_get_f32(q + 20); out[i].qz = sntv_get_f32(q + 24);
      }
      return n;
    }
    off += 4u + len;
  }
  return 0;
}

#ifdef __cplusplus
}
#endif

#endif /* SNTV_H */

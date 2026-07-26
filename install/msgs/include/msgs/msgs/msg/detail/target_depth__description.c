// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from msgs:msg/TargetDepth.idl
// generated code does not contain a copyright notice

#include "msgs/msg/detail/target_depth__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_msgs
const rosidl_type_hash_t *
msgs__msg__TargetDepth__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x9c, 0x04, 0xa4, 0x5e, 0x3c, 0x25, 0x46, 0x11,
      0xb2, 0x44, 0xc5, 0xc0, 0x8d, 0xc0, 0x67, 0xb6,
      0xfd, 0xc4, 0xfb, 0x0c, 0xc6, 0xb8, 0x1c, 0x31,
      0x62, 0x93, 0xfc, 0x98, 0x65, 0x53, 0x3d, 0xf1,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char msgs__msg__TargetDepth__TYPE_NAME[] = "msgs/msg/TargetDepth";

// Define type names, field names, and default values
static char msgs__msg__TargetDepth__FIELD_NAME__depth[] = "depth";

static rosidl_runtime_c__type_description__Field msgs__msg__TargetDepth__FIELDS[] = {
  {
    {msgs__msg__TargetDepth__FIELD_NAME__depth, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
msgs__msg__TargetDepth__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {msgs__msg__TargetDepth__TYPE_NAME, 20, 20},
      {msgs__msg__TargetDepth__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float64 depth";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
msgs__msg__TargetDepth__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {msgs__msg__TargetDepth__TYPE_NAME, 20, 20},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 14, 14},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
msgs__msg__TargetDepth__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *msgs__msg__TargetDepth__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

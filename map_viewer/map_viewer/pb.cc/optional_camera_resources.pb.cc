// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: vi-map/optional_camera_resources.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "vi-map/optional_camera_resources.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace opt_cam_res {
namespace proto {

namespace {

const ::google::protobuf::Descriptor* OptionalCameraResources_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  OptionalCameraResources_reflection_ = NULL;
const ::google::protobuf::Descriptor* CamerasWithExtrinsics_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  CamerasWithExtrinsics_reflection_ = NULL;

}  // namespace


void protobuf_AssignDesc_vi_2dmap_2foptional_5fcamera_5fresources_2eproto() {
  protobuf_AddDesc_vi_2dmap_2foptional_5fcamera_5fresources_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "vi-map/optional_camera_resources.proto");
  GOOGLE_CHECK(file != NULL);
  OptionalCameraResources_descriptor_ = file->message_type(0);
  static const int OptionalCameraResources_offsets_[4] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(OptionalCameraResources, camera_id_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(OptionalCameraResources, timestamp_ns_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(OptionalCameraResources, resource_id_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(OptionalCameraResources, resource_type_),
  };
  OptionalCameraResources_reflection_ =
    new ::google::protobuf::internal::GeneratedMessageReflection(
      OptionalCameraResources_descriptor_,
      OptionalCameraResources::default_instance_,
      OptionalCameraResources_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(OptionalCameraResources, _has_bits_[0]),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(OptionalCameraResources, _unknown_fields_),
      -1,
      ::google::protobuf::DescriptorPool::generated_pool(),
      ::google::protobuf::MessageFactory::generated_factory(),
      sizeof(OptionalCameraResources));
  CamerasWithExtrinsics_descriptor_ = file->message_type(1);
  static const int CamerasWithExtrinsics_offsets_[3] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(CamerasWithExtrinsics, camera_id_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(CamerasWithExtrinsics, t_c_b_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(CamerasWithExtrinsics, camera_),
  };
  CamerasWithExtrinsics_reflection_ =
    new ::google::protobuf::internal::GeneratedMessageReflection(
      CamerasWithExtrinsics_descriptor_,
      CamerasWithExtrinsics::default_instance_,
      CamerasWithExtrinsics_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(CamerasWithExtrinsics, _has_bits_[0]),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(CamerasWithExtrinsics, _unknown_fields_),
      -1,
      ::google::protobuf::DescriptorPool::generated_pool(),
      ::google::protobuf::MessageFactory::generated_factory(),
      sizeof(CamerasWithExtrinsics));
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_vi_2dmap_2foptional_5fcamera_5fresources_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
    OptionalCameraResources_descriptor_, &OptionalCameraResources::default_instance());
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
    CamerasWithExtrinsics_descriptor_, &CamerasWithExtrinsics::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_vi_2dmap_2foptional_5fcamera_5fresources_2eproto() {
  delete OptionalCameraResources::default_instance_;
  delete OptionalCameraResources_reflection_;
  delete CamerasWithExtrinsics::default_instance_;
  delete CamerasWithExtrinsics_reflection_;
}

void protobuf_AddDesc_vi_2dmap_2foptional_5fcamera_5fresources_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::aslam::proto::protobuf_AddDesc_aslam_2dserialization_2fcamera_2eproto();
  ::common::proto::protobuf_AddDesc_maplab_2dcommon_2fid_2eproto();
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n&vi-map/optional_camera_resources.proto"
    "\022\021opt_cam_res.proto\032 aslam-serialization"
    "/camera.proto\032\026maplab-common/id.proto\"\222\001"
    "\n\027OptionalCameraResources\022#\n\tcamera_id\030\001"
    " \001(\0132\020.common.proto.Id\022\024\n\014timestamp_ns\030\002"
    " \001(\003\022%\n\013resource_id\030\003 \001(\0132\020.common.proto"
    ".Id\022\025\n\rresource_type\030\004 \001(\005\"p\n\025CamerasWit"
    "hExtrinsics\022#\n\tcamera_id\030\001 \001(\0132\020.common."
    "proto.Id\022\r\n\005T_C_B\030\002 \003(\001\022#\n\006camera\030\003 \001(\0132"
    "\023.aslam.proto.Camera", 380);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "vi-map/optional_camera_resources.proto", &protobuf_RegisterTypes);
  OptionalCameraResources::default_instance_ = new OptionalCameraResources();
  CamerasWithExtrinsics::default_instance_ = new CamerasWithExtrinsics();
  OptionalCameraResources::default_instance_->InitAsDefaultInstance();
  CamerasWithExtrinsics::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_vi_2dmap_2foptional_5fcamera_5fresources_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_vi_2dmap_2foptional_5fcamera_5fresources_2eproto {
  StaticDescriptorInitializer_vi_2dmap_2foptional_5fcamera_5fresources_2eproto() {
    protobuf_AddDesc_vi_2dmap_2foptional_5fcamera_5fresources_2eproto();
  }
} static_descriptor_initializer_vi_2dmap_2foptional_5fcamera_5fresources_2eproto_;

// ===================================================================

#ifndef _MSC_VER
const int OptionalCameraResources::kCameraIdFieldNumber;
const int OptionalCameraResources::kTimestampNsFieldNumber;
const int OptionalCameraResources::kResourceIdFieldNumber;
const int OptionalCameraResources::kResourceTypeFieldNumber;
#endif  // !_MSC_VER

OptionalCameraResources::OptionalCameraResources()
  : ::google::protobuf::Message() {
  SharedCtor();
  // @@protoc_insertion_point(constructor:opt_cam_res.proto.OptionalCameraResources)
}

void OptionalCameraResources::InitAsDefaultInstance() {
  camera_id_ = const_cast< ::common::proto::Id*>(&::common::proto::Id::default_instance());
  resource_id_ = const_cast< ::common::proto::Id*>(&::common::proto::Id::default_instance());
}

OptionalCameraResources::OptionalCameraResources(const OptionalCameraResources& from)
  : ::google::protobuf::Message() {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:opt_cam_res.proto.OptionalCameraResources)
}

void OptionalCameraResources::SharedCtor() {
  _cached_size_ = 0;
  camera_id_ = NULL;
  timestamp_ns_ = GOOGLE_LONGLONG(0);
  resource_id_ = NULL;
  resource_type_ = 0;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

OptionalCameraResources::~OptionalCameraResources() {
  // @@protoc_insertion_point(destructor:opt_cam_res.proto.OptionalCameraResources)
  SharedDtor();
}

void OptionalCameraResources::SharedDtor() {
  if (this != default_instance_) {
    delete camera_id_;
    delete resource_id_;
  }
}

void OptionalCameraResources::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* OptionalCameraResources::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return OptionalCameraResources_descriptor_;
}

const OptionalCameraResources& OptionalCameraResources::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_vi_2dmap_2foptional_5fcamera_5fresources_2eproto();
  return *default_instance_;
}

OptionalCameraResources* OptionalCameraResources::default_instance_ = NULL;

OptionalCameraResources* OptionalCameraResources::New() const {
  return new OptionalCameraResources;
}

void OptionalCameraResources::Clear() {
  if (_has_bits_[0 / 32] & 15) {
    if (has_camera_id()) {
      if (camera_id_ != NULL) camera_id_->::common::proto::Id::Clear();
    }
    timestamp_ns_ = GOOGLE_LONGLONG(0);
    if (has_resource_id()) {
      if (resource_id_ != NULL) resource_id_->::common::proto::Id::Clear();
    }
    resource_type_ = 0;
  }
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  mutable_unknown_fields()->Clear();
}

bool OptionalCameraResources::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:opt_cam_res.proto.OptionalCameraResources)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional .common.proto.Id camera_id = 1;
      case 1: {
        if (tag == 10) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_camera_id()));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(16)) goto parse_timestamp_ns;
        break;
      }

      // optional int64 timestamp_ns = 2;
      case 2: {
        if (tag == 16) {
         parse_timestamp_ns:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int64, ::google::protobuf::internal::WireFormatLite::TYPE_INT64>(
                 input, &timestamp_ns_)));
          set_has_timestamp_ns();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(26)) goto parse_resource_id;
        break;
      }

      // optional .common.proto.Id resource_id = 3;
      case 3: {
        if (tag == 26) {
         parse_resource_id:
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_resource_id()));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(32)) goto parse_resource_type;
        break;
      }

      // optional int32 resource_type = 4;
      case 4: {
        if (tag == 32) {
         parse_resource_type:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &resource_type_)));
          set_has_resource_type();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectAtEnd()) goto success;
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:opt_cam_res.proto.OptionalCameraResources)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:opt_cam_res.proto.OptionalCameraResources)
  return false;
#undef DO_
}

void OptionalCameraResources::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:opt_cam_res.proto.OptionalCameraResources)
  // optional .common.proto.Id camera_id = 1;
  if (has_camera_id()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, this->camera_id(), output);
  }

  // optional int64 timestamp_ns = 2;
  if (has_timestamp_ns()) {
    ::google::protobuf::internal::WireFormatLite::WriteInt64(2, this->timestamp_ns(), output);
  }

  // optional .common.proto.Id resource_id = 3;
  if (has_resource_id()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      3, this->resource_id(), output);
  }

  // optional int32 resource_type = 4;
  if (has_resource_type()) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(4, this->resource_type(), output);
  }

  if (!unknown_fields().empty()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:opt_cam_res.proto.OptionalCameraResources)
}

::google::protobuf::uint8* OptionalCameraResources::SerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:opt_cam_res.proto.OptionalCameraResources)
  // optional .common.proto.Id camera_id = 1;
  if (has_camera_id()) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteMessageNoVirtualToArray(
        1, this->camera_id(), target);
  }

  // optional int64 timestamp_ns = 2;
  if (has_timestamp_ns()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt64ToArray(2, this->timestamp_ns(), target);
  }

  // optional .common.proto.Id resource_id = 3;
  if (has_resource_id()) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteMessageNoVirtualToArray(
        3, this->resource_id(), target);
  }

  // optional int32 resource_type = 4;
  if (has_resource_type()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(4, this->resource_type(), target);
  }

  if (!unknown_fields().empty()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:opt_cam_res.proto.OptionalCameraResources)
  return target;
}

int OptionalCameraResources::ByteSize() const {
  int total_size = 0;

  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    // optional .common.proto.Id camera_id = 1;
    if (has_camera_id()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
          this->camera_id());
    }

    // optional int64 timestamp_ns = 2;
    if (has_timestamp_ns()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int64Size(
          this->timestamp_ns());
    }

    // optional .common.proto.Id resource_id = 3;
    if (has_resource_id()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
          this->resource_id());
    }

    // optional int32 resource_type = 4;
    if (has_resource_type()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
          this->resource_type());
    }

  }
  if (!unknown_fields().empty()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void OptionalCameraResources::MergeFrom(const ::google::protobuf::Message& from) {
  GOOGLE_CHECK_NE(&from, this);
  const OptionalCameraResources* source =
    ::google::protobuf::internal::dynamic_cast_if_available<const OptionalCameraResources*>(
      &from);
  if (source == NULL) {
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
    MergeFrom(*source);
  }
}

void OptionalCameraResources::MergeFrom(const OptionalCameraResources& from) {
  GOOGLE_CHECK_NE(&from, this);
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_camera_id()) {
      mutable_camera_id()->::common::proto::Id::MergeFrom(from.camera_id());
    }
    if (from.has_timestamp_ns()) {
      set_timestamp_ns(from.timestamp_ns());
    }
    if (from.has_resource_id()) {
      mutable_resource_id()->::common::proto::Id::MergeFrom(from.resource_id());
    }
    if (from.has_resource_type()) {
      set_resource_type(from.resource_type());
    }
  }
  mutable_unknown_fields()->MergeFrom(from.unknown_fields());
}

void OptionalCameraResources::CopyFrom(const ::google::protobuf::Message& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void OptionalCameraResources::CopyFrom(const OptionalCameraResources& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool OptionalCameraResources::IsInitialized() const {

  return true;
}

void OptionalCameraResources::Swap(OptionalCameraResources* other) {
  if (other != this) {
    std::swap(camera_id_, other->camera_id_);
    std::swap(timestamp_ns_, other->timestamp_ns_);
    std::swap(resource_id_, other->resource_id_);
    std::swap(resource_type_, other->resource_type_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    _unknown_fields_.Swap(&other->_unknown_fields_);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::google::protobuf::Metadata OptionalCameraResources::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = OptionalCameraResources_descriptor_;
  metadata.reflection = OptionalCameraResources_reflection_;
  return metadata;
}


// ===================================================================

#ifndef _MSC_VER
const int CamerasWithExtrinsics::kCameraIdFieldNumber;
const int CamerasWithExtrinsics::kTCBFieldNumber;
const int CamerasWithExtrinsics::kCameraFieldNumber;
#endif  // !_MSC_VER

CamerasWithExtrinsics::CamerasWithExtrinsics()
  : ::google::protobuf::Message() {
  SharedCtor();
  // @@protoc_insertion_point(constructor:opt_cam_res.proto.CamerasWithExtrinsics)
}

void CamerasWithExtrinsics::InitAsDefaultInstance() {
  camera_id_ = const_cast< ::common::proto::Id*>(&::common::proto::Id::default_instance());
  camera_ = const_cast< ::aslam::proto::Camera*>(&::aslam::proto::Camera::default_instance());
}

CamerasWithExtrinsics::CamerasWithExtrinsics(const CamerasWithExtrinsics& from)
  : ::google::protobuf::Message() {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:opt_cam_res.proto.CamerasWithExtrinsics)
}

void CamerasWithExtrinsics::SharedCtor() {
  _cached_size_ = 0;
  camera_id_ = NULL;
  camera_ = NULL;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

CamerasWithExtrinsics::~CamerasWithExtrinsics() {
  // @@protoc_insertion_point(destructor:opt_cam_res.proto.CamerasWithExtrinsics)
  SharedDtor();
}

void CamerasWithExtrinsics::SharedDtor() {
  if (this != default_instance_) {
    delete camera_id_;
    delete camera_;
  }
}

void CamerasWithExtrinsics::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* CamerasWithExtrinsics::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return CamerasWithExtrinsics_descriptor_;
}

const CamerasWithExtrinsics& CamerasWithExtrinsics::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_vi_2dmap_2foptional_5fcamera_5fresources_2eproto();
  return *default_instance_;
}

CamerasWithExtrinsics* CamerasWithExtrinsics::default_instance_ = NULL;

CamerasWithExtrinsics* CamerasWithExtrinsics::New() const {
  return new CamerasWithExtrinsics;
}

void CamerasWithExtrinsics::Clear() {
  if (_has_bits_[0 / 32] & 5) {
    if (has_camera_id()) {
      if (camera_id_ != NULL) camera_id_->::common::proto::Id::Clear();
    }
    if (has_camera()) {
      if (camera_ != NULL) camera_->::aslam::proto::Camera::Clear();
    }
  }
  t_c_b_.Clear();
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  mutable_unknown_fields()->Clear();
}

bool CamerasWithExtrinsics::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:opt_cam_res.proto.CamerasWithExtrinsics)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional .common.proto.Id camera_id = 1;
      case 1: {
        if (tag == 10) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_camera_id()));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(17)) goto parse_T_C_B;
        break;
      }

      // repeated double T_C_B = 2;
      case 2: {
        if (tag == 17) {
         parse_T_C_B:
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 1, 17, input, this->mutable_t_c_b())));
        } else if (tag == 18) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitiveNoInline<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, this->mutable_t_c_b())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(17)) goto parse_T_C_B;
        if (input->ExpectTag(26)) goto parse_camera;
        break;
      }

      // optional .aslam.proto.Camera camera = 3;
      case 3: {
        if (tag == 26) {
         parse_camera:
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_camera()));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectAtEnd()) goto success;
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:opt_cam_res.proto.CamerasWithExtrinsics)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:opt_cam_res.proto.CamerasWithExtrinsics)
  return false;
#undef DO_
}

void CamerasWithExtrinsics::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:opt_cam_res.proto.CamerasWithExtrinsics)
  // optional .common.proto.Id camera_id = 1;
  if (has_camera_id()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, this->camera_id(), output);
  }

  // repeated double T_C_B = 2;
  for (int i = 0; i < this->t_c_b_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(
      2, this->t_c_b(i), output);
  }

  // optional .aslam.proto.Camera camera = 3;
  if (has_camera()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      3, this->camera(), output);
  }

  if (!unknown_fields().empty()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:opt_cam_res.proto.CamerasWithExtrinsics)
}

::google::protobuf::uint8* CamerasWithExtrinsics::SerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:opt_cam_res.proto.CamerasWithExtrinsics)
  // optional .common.proto.Id camera_id = 1;
  if (has_camera_id()) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteMessageNoVirtualToArray(
        1, this->camera_id(), target);
  }

  // repeated double T_C_B = 2;
  for (int i = 0; i < this->t_c_b_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteDoubleToArray(2, this->t_c_b(i), target);
  }

  // optional .aslam.proto.Camera camera = 3;
  if (has_camera()) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteMessageNoVirtualToArray(
        3, this->camera(), target);
  }

  if (!unknown_fields().empty()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:opt_cam_res.proto.CamerasWithExtrinsics)
  return target;
}

int CamerasWithExtrinsics::ByteSize() const {
  int total_size = 0;

  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    // optional .common.proto.Id camera_id = 1;
    if (has_camera_id()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
          this->camera_id());
    }

    // optional .aslam.proto.Camera camera = 3;
    if (has_camera()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
          this->camera());
    }

  }
  // repeated double T_C_B = 2;
  {
    int data_size = 0;
    data_size = 8 * this->t_c_b_size();
    total_size += 1 * this->t_c_b_size() + data_size;
  }

  if (!unknown_fields().empty()) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        unknown_fields());
  }
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = total_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void CamerasWithExtrinsics::MergeFrom(const ::google::protobuf::Message& from) {
  GOOGLE_CHECK_NE(&from, this);
  const CamerasWithExtrinsics* source =
    ::google::protobuf::internal::dynamic_cast_if_available<const CamerasWithExtrinsics*>(
      &from);
  if (source == NULL) {
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
    MergeFrom(*source);
  }
}

void CamerasWithExtrinsics::MergeFrom(const CamerasWithExtrinsics& from) {
  GOOGLE_CHECK_NE(&from, this);
  t_c_b_.MergeFrom(from.t_c_b_);
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_camera_id()) {
      mutable_camera_id()->::common::proto::Id::MergeFrom(from.camera_id());
    }
    if (from.has_camera()) {
      mutable_camera()->::aslam::proto::Camera::MergeFrom(from.camera());
    }
  }
  mutable_unknown_fields()->MergeFrom(from.unknown_fields());
}

void CamerasWithExtrinsics::CopyFrom(const ::google::protobuf::Message& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void CamerasWithExtrinsics::CopyFrom(const CamerasWithExtrinsics& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool CamerasWithExtrinsics::IsInitialized() const {

  return true;
}

void CamerasWithExtrinsics::Swap(CamerasWithExtrinsics* other) {
  if (other != this) {
    std::swap(camera_id_, other->camera_id_);
    t_c_b_.Swap(&other->t_c_b_);
    std::swap(camera_, other->camera_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    _unknown_fields_.Swap(&other->_unknown_fields_);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::google::protobuf::Metadata CamerasWithExtrinsics::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = CamerasWithExtrinsics_descriptor_;
  metadata.reflection = CamerasWithExtrinsics_reflection_;
  return metadata;
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace opt_cam_res

// @@protoc_insertion_point(global_scope)

// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: maplab-common/id.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "maplab-common/id.pb.h"

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

namespace common {
namespace proto {

namespace {

const ::google::protobuf::Descriptor* Id_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  Id_reflection_ = NULL;

}  // namespace


void protobuf_AssignDesc_maplab_2dcommon_2fid_2eproto() {
  protobuf_AddDesc_maplab_2dcommon_2fid_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "maplab-common/id.proto");
  GOOGLE_CHECK(file != NULL);
  Id_descriptor_ = file->message_type(0);
  static const int Id_offsets_[1] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Id, uint_),
  };
  Id_reflection_ =
    new ::google::protobuf::internal::GeneratedMessageReflection(
      Id_descriptor_,
      Id::default_instance_,
      Id_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Id, _has_bits_[0]),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Id, _unknown_fields_),
      -1,
      ::google::protobuf::DescriptorPool::generated_pool(),
      ::google::protobuf::MessageFactory::generated_factory(),
      sizeof(Id));
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_maplab_2dcommon_2fid_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
    Id_descriptor_, &Id::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_maplab_2dcommon_2fid_2eproto() {
  delete Id::default_instance_;
  delete Id_reflection_;
}

void protobuf_AddDesc_maplab_2dcommon_2fid_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\026maplab-common/id.proto\022\014common.proto\"\022"
    "\n\002Id\022\014\n\004uint\030\001 \003(\004", 58);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "maplab-common/id.proto", &protobuf_RegisterTypes);
  Id::default_instance_ = new Id();
  Id::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_maplab_2dcommon_2fid_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_maplab_2dcommon_2fid_2eproto {
  StaticDescriptorInitializer_maplab_2dcommon_2fid_2eproto() {
    protobuf_AddDesc_maplab_2dcommon_2fid_2eproto();
  }
} static_descriptor_initializer_maplab_2dcommon_2fid_2eproto_;

// ===================================================================

#ifndef _MSC_VER
const int Id::kUintFieldNumber;
#endif  // !_MSC_VER

Id::Id()
  : ::google::protobuf::Message() {
  SharedCtor();
  // @@protoc_insertion_point(constructor:common.proto.Id)
}

void Id::InitAsDefaultInstance() {
}

Id::Id(const Id& from)
  : ::google::protobuf::Message() {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:common.proto.Id)
}

void Id::SharedCtor() {
  _cached_size_ = 0;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

Id::~Id() {
  // @@protoc_insertion_point(destructor:common.proto.Id)
  SharedDtor();
}

void Id::SharedDtor() {
  if (this != default_instance_) {
  }
}

void Id::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* Id::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return Id_descriptor_;
}

const Id& Id::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_maplab_2dcommon_2fid_2eproto();
  return *default_instance_;
}

Id* Id::default_instance_ = NULL;

Id* Id::New() const {
  return new Id;
}

void Id::Clear() {
  uint_.Clear();
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  mutable_unknown_fields()->Clear();
}

bool Id::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:common.proto.Id)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // repeated uint64 uint = 1;
      case 1: {
        if (tag == 8) {
         parse_uint:
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitive<
                   ::google::protobuf::uint64, ::google::protobuf::internal::WireFormatLite::TYPE_UINT64>(
                 1, 8, input, this->mutable_uint())));
        } else if (tag == 10) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitiveNoInline<
                   ::google::protobuf::uint64, ::google::protobuf::internal::WireFormatLite::TYPE_UINT64>(
                 input, this->mutable_uint())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(8)) goto parse_uint;
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
  // @@protoc_insertion_point(parse_success:common.proto.Id)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:common.proto.Id)
  return false;
#undef DO_
}

void Id::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:common.proto.Id)
  // repeated uint64 uint = 1;
  for (int i = 0; i < this->uint_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteUInt64(
      1, this->uint(i), output);
  }

  if (!unknown_fields().empty()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:common.proto.Id)
}

::google::protobuf::uint8* Id::SerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:common.proto.Id)
  // repeated uint64 uint = 1;
  for (int i = 0; i < this->uint_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteUInt64ToArray(1, this->uint(i), target);
  }

  if (!unknown_fields().empty()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:common.proto.Id)
  return target;
}

int Id::ByteSize() const {
  int total_size = 0;

  // repeated uint64 uint = 1;
  {
    int data_size = 0;
    for (int i = 0; i < this->uint_size(); i++) {
      data_size += ::google::protobuf::internal::WireFormatLite::
        UInt64Size(this->uint(i));
    }
    total_size += 1 * this->uint_size() + data_size;
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

void Id::MergeFrom(const ::google::protobuf::Message& from) {
  GOOGLE_CHECK_NE(&from, this);
  const Id* source =
    ::google::protobuf::internal::dynamic_cast_if_available<const Id*>(
      &from);
  if (source == NULL) {
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
    MergeFrom(*source);
  }
}

void Id::MergeFrom(const Id& from) {
  GOOGLE_CHECK_NE(&from, this);
  uint_.MergeFrom(from.uint_);
  mutable_unknown_fields()->MergeFrom(from.unknown_fields());
}

void Id::CopyFrom(const ::google::protobuf::Message& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Id::CopyFrom(const Id& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Id::IsInitialized() const {

  return true;
}

void Id::Swap(Id* other) {
  if (other != this) {
    uint_.Swap(&other->uint_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    _unknown_fields_.Swap(&other->_unknown_fields_);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::google::protobuf::Metadata Id::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = Id_descriptor_;
  metadata.reflection = Id_reflection_;
  return metadata;
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace common

// @@protoc_insertion_point(global_scope)

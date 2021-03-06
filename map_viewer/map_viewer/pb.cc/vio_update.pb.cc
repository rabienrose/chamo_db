// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: vio-common/vio_update.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "vio-common/vio_update.pb.h"

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

namespace vio {
namespace proto {

namespace {

const ::google::protobuf::Descriptor* VioUpdate_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  VioUpdate_reflection_ = NULL;

}  // namespace


void protobuf_AssignDesc_vio_2dcommon_2fvio_5fupdate_2eproto() {
  protobuf_AddDesc_vio_2dcommon_2fvio_5fupdate_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "vio-common/vio_update.proto");
  GOOGLE_CHECK(file != NULL);
  VioUpdate_descriptor_ = file->message_type(0);
  static const int VioUpdate_offsets_[7] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(VioUpdate, timestamp_ns_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(VioUpdate, vio_state_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(VioUpdate, vio_update_type_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(VioUpdate, keyframe_and_imudata_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(VioUpdate, vinode_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(VioUpdate, localization_state_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(VioUpdate, t_g_m_),
  };
  VioUpdate_reflection_ =
    new ::google::protobuf::internal::GeneratedMessageReflection(
      VioUpdate_descriptor_,
      VioUpdate::default_instance_,
      VioUpdate_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(VioUpdate, _has_bits_[0]),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(VioUpdate, _unknown_fields_),
      -1,
      ::google::protobuf::DescriptorPool::generated_pool(),
      ::google::protobuf::MessageFactory::generated_factory(),
      sizeof(VioUpdate));
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_vio_2dcommon_2fvio_5fupdate_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
    VioUpdate_descriptor_, &VioUpdate::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_vio_2dcommon_2fvio_5fupdate_2eproto() {
  delete VioUpdate::default_instance_;
  delete VioUpdate_reflection_;
}

void protobuf_AddDesc_vio_2dcommon_2fvio_5fupdate_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::vio::proto::protobuf_AddDesc_vio_2dcommon_2fvio_5ftypes_2eproto();
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\033vio-common/vio_update.proto\022\tvio.proto"
    "\032\032vio-common/vio_types.proto\"\340\001\n\tVioUpda"
    "te\022\024\n\014timestamp_ns\030\001 \001(\003\022\021\n\tvio_state\030\002 "
    "\001(\005\022\027\n\017vio_update_type\030\003 \001(\005\022>\n\024keyframe"
    "_and_imudata\030\004 \001(\0132 .vio.proto.Synchroni"
    "zedNFrameImu\022&\n\006vinode\030\005 \001(\0132\026.vio.proto"
    ".ViNodeState\022\032\n\022localization_state\030\006 \001(\005"
    "\022\r\n\005T_G_M\030\007 \003(\001", 295);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "vio-common/vio_update.proto", &protobuf_RegisterTypes);
  VioUpdate::default_instance_ = new VioUpdate();
  VioUpdate::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_vio_2dcommon_2fvio_5fupdate_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_vio_2dcommon_2fvio_5fupdate_2eproto {
  StaticDescriptorInitializer_vio_2dcommon_2fvio_5fupdate_2eproto() {
    protobuf_AddDesc_vio_2dcommon_2fvio_5fupdate_2eproto();
  }
} static_descriptor_initializer_vio_2dcommon_2fvio_5fupdate_2eproto_;

// ===================================================================

#ifndef _MSC_VER
const int VioUpdate::kTimestampNsFieldNumber;
const int VioUpdate::kVioStateFieldNumber;
const int VioUpdate::kVioUpdateTypeFieldNumber;
const int VioUpdate::kKeyframeAndImudataFieldNumber;
const int VioUpdate::kVinodeFieldNumber;
const int VioUpdate::kLocalizationStateFieldNumber;
const int VioUpdate::kTGMFieldNumber;
#endif  // !_MSC_VER

VioUpdate::VioUpdate()
  : ::google::protobuf::Message() {
  SharedCtor();
  // @@protoc_insertion_point(constructor:vio.proto.VioUpdate)
}

void VioUpdate::InitAsDefaultInstance() {
  keyframe_and_imudata_ = const_cast< ::vio::proto::SynchronizedNFrameImu*>(&::vio::proto::SynchronizedNFrameImu::default_instance());
  vinode_ = const_cast< ::vio::proto::ViNodeState*>(&::vio::proto::ViNodeState::default_instance());
}

VioUpdate::VioUpdate(const VioUpdate& from)
  : ::google::protobuf::Message() {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:vio.proto.VioUpdate)
}

void VioUpdate::SharedCtor() {
  _cached_size_ = 0;
  timestamp_ns_ = GOOGLE_LONGLONG(0);
  vio_state_ = 0;
  vio_update_type_ = 0;
  keyframe_and_imudata_ = NULL;
  vinode_ = NULL;
  localization_state_ = 0;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

VioUpdate::~VioUpdate() {
  // @@protoc_insertion_point(destructor:vio.proto.VioUpdate)
  SharedDtor();
}

void VioUpdate::SharedDtor() {
  if (this != default_instance_) {
    delete keyframe_and_imudata_;
    delete vinode_;
  }
}

void VioUpdate::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* VioUpdate::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return VioUpdate_descriptor_;
}

const VioUpdate& VioUpdate::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_vio_2dcommon_2fvio_5fupdate_2eproto();
  return *default_instance_;
}

VioUpdate* VioUpdate::default_instance_ = NULL;

VioUpdate* VioUpdate::New() const {
  return new VioUpdate;
}

void VioUpdate::Clear() {
#define OFFSET_OF_FIELD_(f) (reinterpret_cast<char*>(      \
  &reinterpret_cast<VioUpdate*>(16)->f) - \
   reinterpret_cast<char*>(16))

#define ZR_(first, last) do {                              \
    size_t f = OFFSET_OF_FIELD_(first);                    \
    size_t n = OFFSET_OF_FIELD_(last) - f + sizeof(last);  \
    ::memset(&first, 0, n);                                \
  } while (0)

  if (_has_bits_[0 / 32] & 63) {
    ZR_(timestamp_ns_, vio_update_type_);
    if (has_keyframe_and_imudata()) {
      if (keyframe_and_imudata_ != NULL) keyframe_and_imudata_->::vio::proto::SynchronizedNFrameImu::Clear();
    }
    if (has_vinode()) {
      if (vinode_ != NULL) vinode_->::vio::proto::ViNodeState::Clear();
    }
    localization_state_ = 0;
  }

#undef OFFSET_OF_FIELD_
#undef ZR_

  t_g_m_.Clear();
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  mutable_unknown_fields()->Clear();
}

bool VioUpdate::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:vio.proto.VioUpdate)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional int64 timestamp_ns = 1;
      case 1: {
        if (tag == 8) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int64, ::google::protobuf::internal::WireFormatLite::TYPE_INT64>(
                 input, &timestamp_ns_)));
          set_has_timestamp_ns();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(16)) goto parse_vio_state;
        break;
      }

      // optional int32 vio_state = 2;
      case 2: {
        if (tag == 16) {
         parse_vio_state:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &vio_state_)));
          set_has_vio_state();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(24)) goto parse_vio_update_type;
        break;
      }

      // optional int32 vio_update_type = 3;
      case 3: {
        if (tag == 24) {
         parse_vio_update_type:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &vio_update_type_)));
          set_has_vio_update_type();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(34)) goto parse_keyframe_and_imudata;
        break;
      }

      // optional .vio.proto.SynchronizedNFrameImu keyframe_and_imudata = 4;
      case 4: {
        if (tag == 34) {
         parse_keyframe_and_imudata:
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_keyframe_and_imudata()));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(42)) goto parse_vinode;
        break;
      }

      // optional .vio.proto.ViNodeState vinode = 5;
      case 5: {
        if (tag == 42) {
         parse_vinode:
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_vinode()));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(48)) goto parse_localization_state;
        break;
      }

      // optional int32 localization_state = 6;
      case 6: {
        if (tag == 48) {
         parse_localization_state:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &localization_state_)));
          set_has_localization_state();
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(57)) goto parse_T_G_M;
        break;
      }

      // repeated double T_G_M = 7;
      case 7: {
        if (tag == 57) {
         parse_T_G_M:
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 1, 57, input, this->mutable_t_g_m())));
        } else if (tag == 58) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitiveNoInline<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, this->mutable_t_g_m())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(57)) goto parse_T_G_M;
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
  // @@protoc_insertion_point(parse_success:vio.proto.VioUpdate)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:vio.proto.VioUpdate)
  return false;
#undef DO_
}

void VioUpdate::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:vio.proto.VioUpdate)
  // optional int64 timestamp_ns = 1;
  if (has_timestamp_ns()) {
    ::google::protobuf::internal::WireFormatLite::WriteInt64(1, this->timestamp_ns(), output);
  }

  // optional int32 vio_state = 2;
  if (has_vio_state()) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(2, this->vio_state(), output);
  }

  // optional int32 vio_update_type = 3;
  if (has_vio_update_type()) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(3, this->vio_update_type(), output);
  }

  // optional .vio.proto.SynchronizedNFrameImu keyframe_and_imudata = 4;
  if (has_keyframe_and_imudata()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      4, this->keyframe_and_imudata(), output);
  }

  // optional .vio.proto.ViNodeState vinode = 5;
  if (has_vinode()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      5, this->vinode(), output);
  }

  // optional int32 localization_state = 6;
  if (has_localization_state()) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(6, this->localization_state(), output);
  }

  // repeated double T_G_M = 7;
  for (int i = 0; i < this->t_g_m_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(
      7, this->t_g_m(i), output);
  }

  if (!unknown_fields().empty()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:vio.proto.VioUpdate)
}

::google::protobuf::uint8* VioUpdate::SerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:vio.proto.VioUpdate)
  // optional int64 timestamp_ns = 1;
  if (has_timestamp_ns()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt64ToArray(1, this->timestamp_ns(), target);
  }

  // optional int32 vio_state = 2;
  if (has_vio_state()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(2, this->vio_state(), target);
  }

  // optional int32 vio_update_type = 3;
  if (has_vio_update_type()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(3, this->vio_update_type(), target);
  }

  // optional .vio.proto.SynchronizedNFrameImu keyframe_and_imudata = 4;
  if (has_keyframe_and_imudata()) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteMessageNoVirtualToArray(
        4, this->keyframe_and_imudata(), target);
  }

  // optional .vio.proto.ViNodeState vinode = 5;
  if (has_vinode()) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteMessageNoVirtualToArray(
        5, this->vinode(), target);
  }

  // optional int32 localization_state = 6;
  if (has_localization_state()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(6, this->localization_state(), target);
  }

  // repeated double T_G_M = 7;
  for (int i = 0; i < this->t_g_m_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteDoubleToArray(7, this->t_g_m(i), target);
  }

  if (!unknown_fields().empty()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:vio.proto.VioUpdate)
  return target;
}

int VioUpdate::ByteSize() const {
  int total_size = 0;

  if (_has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    // optional int64 timestamp_ns = 1;
    if (has_timestamp_ns()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int64Size(
          this->timestamp_ns());
    }

    // optional int32 vio_state = 2;
    if (has_vio_state()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
          this->vio_state());
    }

    // optional int32 vio_update_type = 3;
    if (has_vio_update_type()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
          this->vio_update_type());
    }

    // optional .vio.proto.SynchronizedNFrameImu keyframe_and_imudata = 4;
    if (has_keyframe_and_imudata()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
          this->keyframe_and_imudata());
    }

    // optional .vio.proto.ViNodeState vinode = 5;
    if (has_vinode()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
          this->vinode());
    }

    // optional int32 localization_state = 6;
    if (has_localization_state()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
          this->localization_state());
    }

  }
  // repeated double T_G_M = 7;
  {
    int data_size = 0;
    data_size = 8 * this->t_g_m_size();
    total_size += 1 * this->t_g_m_size() + data_size;
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

void VioUpdate::MergeFrom(const ::google::protobuf::Message& from) {
  GOOGLE_CHECK_NE(&from, this);
  const VioUpdate* source =
    ::google::protobuf::internal::dynamic_cast_if_available<const VioUpdate*>(
      &from);
  if (source == NULL) {
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
    MergeFrom(*source);
  }
}

void VioUpdate::MergeFrom(const VioUpdate& from) {
  GOOGLE_CHECK_NE(&from, this);
  t_g_m_.MergeFrom(from.t_g_m_);
  if (from._has_bits_[0 / 32] & (0xffu << (0 % 32))) {
    if (from.has_timestamp_ns()) {
      set_timestamp_ns(from.timestamp_ns());
    }
    if (from.has_vio_state()) {
      set_vio_state(from.vio_state());
    }
    if (from.has_vio_update_type()) {
      set_vio_update_type(from.vio_update_type());
    }
    if (from.has_keyframe_and_imudata()) {
      mutable_keyframe_and_imudata()->::vio::proto::SynchronizedNFrameImu::MergeFrom(from.keyframe_and_imudata());
    }
    if (from.has_vinode()) {
      mutable_vinode()->::vio::proto::ViNodeState::MergeFrom(from.vinode());
    }
    if (from.has_localization_state()) {
      set_localization_state(from.localization_state());
    }
  }
  mutable_unknown_fields()->MergeFrom(from.unknown_fields());
}

void VioUpdate::CopyFrom(const ::google::protobuf::Message& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void VioUpdate::CopyFrom(const VioUpdate& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool VioUpdate::IsInitialized() const {

  return true;
}

void VioUpdate::Swap(VioUpdate* other) {
  if (other != this) {
    std::swap(timestamp_ns_, other->timestamp_ns_);
    std::swap(vio_state_, other->vio_state_);
    std::swap(vio_update_type_, other->vio_update_type_);
    std::swap(keyframe_and_imudata_, other->keyframe_and_imudata_);
    std::swap(vinode_, other->vinode_);
    std::swap(localization_state_, other->localization_state_);
    t_g_m_.Swap(&other->t_g_m_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    _unknown_fields_.Swap(&other->_unknown_fields_);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::google::protobuf::Metadata VioUpdate::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = VioUpdate_descriptor_;
  metadata.reflection = VioUpdate_reflection_;
  return metadata;
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace vio

// @@protoc_insertion_point(global_scope)

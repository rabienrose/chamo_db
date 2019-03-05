// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: vio-common/vio_types.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "vio-common/vio_types.pb.h"

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

const ::google::protobuf::Descriptor* SynchronizedNFrameImu_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  SynchronizedNFrameImu_reflection_ = NULL;
const ::google::protobuf::Descriptor* ViNodeState_descriptor_ = NULL;
const ::google::protobuf::internal::GeneratedMessageReflection*
  ViNodeState_reflection_ = NULL;

}  // namespace


void protobuf_AssignDesc_vio_2dcommon_2fvio_5ftypes_2eproto() {
  protobuf_AddDesc_vio_2dcommon_2fvio_5ftypes_2eproto();
  const ::google::protobuf::FileDescriptor* file =
    ::google::protobuf::DescriptorPool::generated_pool()->FindFileByName(
      "vio-common/vio_types.proto");
  GOOGLE_CHECK(file != NULL);
  SynchronizedNFrameImu_descriptor_ = file->message_type(0);
  static const int SynchronizedNFrameImu_offsets_[4] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(SynchronizedNFrameImu, imu_timestamps_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(SynchronizedNFrameImu, imu_measurements_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(SynchronizedNFrameImu, nframe_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(SynchronizedNFrameImu, motion_wrt_last_nframe_),
  };
  SynchronizedNFrameImu_reflection_ =
    new ::google::protobuf::internal::GeneratedMessageReflection(
      SynchronizedNFrameImu_descriptor_,
      SynchronizedNFrameImu::default_instance_,
      SynchronizedNFrameImu_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(SynchronizedNFrameImu, _has_bits_[0]),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(SynchronizedNFrameImu, _unknown_fields_),
      -1,
      ::google::protobuf::DescriptorPool::generated_pool(),
      ::google::protobuf::MessageFactory::generated_factory(),
      sizeof(SynchronizedNFrameImu));
  ViNodeState_descriptor_ = file->message_type(1);
  static const int ViNodeState_offsets_[4] = {
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(ViNodeState, t_w_b_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(ViNodeState, w_v_b_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(ViNodeState, acc_bias_),
    GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(ViNodeState, gyro_bias_),
  };
  ViNodeState_reflection_ =
    new ::google::protobuf::internal::GeneratedMessageReflection(
      ViNodeState_descriptor_,
      ViNodeState::default_instance_,
      ViNodeState_offsets_,
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(ViNodeState, _has_bits_[0]),
      GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(ViNodeState, _unknown_fields_),
      -1,
      ::google::protobuf::DescriptorPool::generated_pool(),
      ::google::protobuf::MessageFactory::generated_factory(),
      sizeof(ViNodeState));
}

namespace {

GOOGLE_PROTOBUF_DECLARE_ONCE(protobuf_AssignDescriptors_once_);
inline void protobuf_AssignDescriptorsOnce() {
  ::google::protobuf::GoogleOnceInit(&protobuf_AssignDescriptors_once_,
                 &protobuf_AssignDesc_vio_2dcommon_2fvio_5ftypes_2eproto);
}

void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
    SynchronizedNFrameImu_descriptor_, &SynchronizedNFrameImu::default_instance());
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedMessage(
    ViNodeState_descriptor_, &ViNodeState::default_instance());
}

}  // namespace

void protobuf_ShutdownFile_vio_2dcommon_2fvio_5ftypes_2eproto() {
  delete SynchronizedNFrameImu::default_instance_;
  delete SynchronizedNFrameImu_reflection_;
  delete ViNodeState::default_instance_;
  delete ViNodeState_reflection_;
}

void protobuf_AddDesc_vio_2dcommon_2fvio_5ftypes_2eproto() {
  static bool already_here = false;
  if (already_here) return;
  already_here = true;
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::aslam::proto::protobuf_AddDesc_aslam_2dserialization_2fvisual_2dframe_2eproto();
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
    "\n\032vio-common/vio_types.proto\022\tvio.proto\032"
    "&aslam-serialization/visual-frame.proto\""
    "\224\001\n\025SynchronizedNFrameImu\022\026\n\016imu_timesta"
    "mps\030\001 \003(\003\022\030\n\020imu_measurements\030\002 \003(\001\022)\n\006n"
    "frame\030\003 \001(\0132\031.aslam.proto.VisualNFrame\022\036"
    "\n\026motion_wrt_last_nframe\030\004 \001(\005\"P\n\013ViNode"
    "State\022\r\n\005T_W_B\030\001 \003(\001\022\r\n\005W_v_B\030\002 \003(\001\022\020\n\010a"
    "cc_bias\030\003 \003(\001\022\021\n\tgyro_bias\030\004 \003(\001", 312);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "vio-common/vio_types.proto", &protobuf_RegisterTypes);
  SynchronizedNFrameImu::default_instance_ = new SynchronizedNFrameImu();
  ViNodeState::default_instance_ = new ViNodeState();
  SynchronizedNFrameImu::default_instance_->InitAsDefaultInstance();
  ViNodeState::default_instance_->InitAsDefaultInstance();
  ::google::protobuf::internal::OnShutdown(&protobuf_ShutdownFile_vio_2dcommon_2fvio_5ftypes_2eproto);
}

// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer_vio_2dcommon_2fvio_5ftypes_2eproto {
  StaticDescriptorInitializer_vio_2dcommon_2fvio_5ftypes_2eproto() {
    protobuf_AddDesc_vio_2dcommon_2fvio_5ftypes_2eproto();
  }
} static_descriptor_initializer_vio_2dcommon_2fvio_5ftypes_2eproto_;

// ===================================================================

#ifndef _MSC_VER
const int SynchronizedNFrameImu::kImuTimestampsFieldNumber;
const int SynchronizedNFrameImu::kImuMeasurementsFieldNumber;
const int SynchronizedNFrameImu::kNframeFieldNumber;
const int SynchronizedNFrameImu::kMotionWrtLastNframeFieldNumber;
#endif  // !_MSC_VER

SynchronizedNFrameImu::SynchronizedNFrameImu()
  : ::google::protobuf::Message() {
  SharedCtor();
  // @@protoc_insertion_point(constructor:vio.proto.SynchronizedNFrameImu)
}

void SynchronizedNFrameImu::InitAsDefaultInstance() {
  nframe_ = const_cast< ::aslam::proto::VisualNFrame*>(&::aslam::proto::VisualNFrame::default_instance());
}

SynchronizedNFrameImu::SynchronizedNFrameImu(const SynchronizedNFrameImu& from)
  : ::google::protobuf::Message() {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:vio.proto.SynchronizedNFrameImu)
}

void SynchronizedNFrameImu::SharedCtor() {
  _cached_size_ = 0;
  nframe_ = NULL;
  motion_wrt_last_nframe_ = 0;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

SynchronizedNFrameImu::~SynchronizedNFrameImu() {
  // @@protoc_insertion_point(destructor:vio.proto.SynchronizedNFrameImu)
  SharedDtor();
}

void SynchronizedNFrameImu::SharedDtor() {
  if (this != default_instance_) {
    delete nframe_;
  }
}

void SynchronizedNFrameImu::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* SynchronizedNFrameImu::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return SynchronizedNFrameImu_descriptor_;
}

const SynchronizedNFrameImu& SynchronizedNFrameImu::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_vio_2dcommon_2fvio_5ftypes_2eproto();
  return *default_instance_;
}

SynchronizedNFrameImu* SynchronizedNFrameImu::default_instance_ = NULL;

SynchronizedNFrameImu* SynchronizedNFrameImu::New() const {
  return new SynchronizedNFrameImu;
}

void SynchronizedNFrameImu::Clear() {
  if (_has_bits_[0 / 32] & 12) {
    if (has_nframe()) {
      if (nframe_ != NULL) nframe_->::aslam::proto::VisualNFrame::Clear();
    }
    motion_wrt_last_nframe_ = 0;
  }
  imu_timestamps_.Clear();
  imu_measurements_.Clear();
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  mutable_unknown_fields()->Clear();
}

bool SynchronizedNFrameImu::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:vio.proto.SynchronizedNFrameImu)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // repeated int64 imu_timestamps = 1;
      case 1: {
        if (tag == 8) {
         parse_imu_timestamps:
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitive<
                   ::google::protobuf::int64, ::google::protobuf::internal::WireFormatLite::TYPE_INT64>(
                 1, 8, input, this->mutable_imu_timestamps())));
        } else if (tag == 10) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitiveNoInline<
                   ::google::protobuf::int64, ::google::protobuf::internal::WireFormatLite::TYPE_INT64>(
                 input, this->mutable_imu_timestamps())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(8)) goto parse_imu_timestamps;
        if (input->ExpectTag(17)) goto parse_imu_measurements;
        break;
      }

      // repeated double imu_measurements = 2;
      case 2: {
        if (tag == 17) {
         parse_imu_measurements:
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 1, 17, input, this->mutable_imu_measurements())));
        } else if (tag == 18) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitiveNoInline<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, this->mutable_imu_measurements())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(17)) goto parse_imu_measurements;
        if (input->ExpectTag(26)) goto parse_nframe;
        break;
      }

      // optional .aslam.proto.VisualNFrame nframe = 3;
      case 3: {
        if (tag == 26) {
         parse_nframe:
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_nframe()));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(32)) goto parse_motion_wrt_last_nframe;
        break;
      }

      // optional int32 motion_wrt_last_nframe = 4;
      case 4: {
        if (tag == 32) {
         parse_motion_wrt_last_nframe:
          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &motion_wrt_last_nframe_)));
          set_has_motion_wrt_last_nframe();
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
  // @@protoc_insertion_point(parse_success:vio.proto.SynchronizedNFrameImu)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:vio.proto.SynchronizedNFrameImu)
  return false;
#undef DO_
}

void SynchronizedNFrameImu::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:vio.proto.SynchronizedNFrameImu)
  // repeated int64 imu_timestamps = 1;
  for (int i = 0; i < this->imu_timestamps_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteInt64(
      1, this->imu_timestamps(i), output);
  }

  // repeated double imu_measurements = 2;
  for (int i = 0; i < this->imu_measurements_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(
      2, this->imu_measurements(i), output);
  }

  // optional .aslam.proto.VisualNFrame nframe = 3;
  if (has_nframe()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      3, this->nframe(), output);
  }

  // optional int32 motion_wrt_last_nframe = 4;
  if (has_motion_wrt_last_nframe()) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(4, this->motion_wrt_last_nframe(), output);
  }

  if (!unknown_fields().empty()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:vio.proto.SynchronizedNFrameImu)
}

::google::protobuf::uint8* SynchronizedNFrameImu::SerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:vio.proto.SynchronizedNFrameImu)
  // repeated int64 imu_timestamps = 1;
  for (int i = 0; i < this->imu_timestamps_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteInt64ToArray(1, this->imu_timestamps(i), target);
  }

  // repeated double imu_measurements = 2;
  for (int i = 0; i < this->imu_measurements_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteDoubleToArray(2, this->imu_measurements(i), target);
  }

  // optional .aslam.proto.VisualNFrame nframe = 3;
  if (has_nframe()) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteMessageNoVirtualToArray(
        3, this->nframe(), target);
  }

  // optional int32 motion_wrt_last_nframe = 4;
  if (has_motion_wrt_last_nframe()) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(4, this->motion_wrt_last_nframe(), target);
  }

  if (!unknown_fields().empty()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:vio.proto.SynchronizedNFrameImu)
  return target;
}

int SynchronizedNFrameImu::ByteSize() const {
  int total_size = 0;

  if (_has_bits_[2 / 32] & (0xffu << (2 % 32))) {
    // optional .aslam.proto.VisualNFrame nframe = 3;
    if (has_nframe()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
          this->nframe());
    }

    // optional int32 motion_wrt_last_nframe = 4;
    if (has_motion_wrt_last_nframe()) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
          this->motion_wrt_last_nframe());
    }

  }
  // repeated int64 imu_timestamps = 1;
  {
    int data_size = 0;
    for (int i = 0; i < this->imu_timestamps_size(); i++) {
      data_size += ::google::protobuf::internal::WireFormatLite::
        Int64Size(this->imu_timestamps(i));
    }
    total_size += 1 * this->imu_timestamps_size() + data_size;
  }

  // repeated double imu_measurements = 2;
  {
    int data_size = 0;
    data_size = 8 * this->imu_measurements_size();
    total_size += 1 * this->imu_measurements_size() + data_size;
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

void SynchronizedNFrameImu::MergeFrom(const ::google::protobuf::Message& from) {
  GOOGLE_CHECK_NE(&from, this);
  const SynchronizedNFrameImu* source =
    ::google::protobuf::internal::dynamic_cast_if_available<const SynchronizedNFrameImu*>(
      &from);
  if (source == NULL) {
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
    MergeFrom(*source);
  }
}

void SynchronizedNFrameImu::MergeFrom(const SynchronizedNFrameImu& from) {
  GOOGLE_CHECK_NE(&from, this);
  imu_timestamps_.MergeFrom(from.imu_timestamps_);
  imu_measurements_.MergeFrom(from.imu_measurements_);
  if (from._has_bits_[2 / 32] & (0xffu << (2 % 32))) {
    if (from.has_nframe()) {
      mutable_nframe()->::aslam::proto::VisualNFrame::MergeFrom(from.nframe());
    }
    if (from.has_motion_wrt_last_nframe()) {
      set_motion_wrt_last_nframe(from.motion_wrt_last_nframe());
    }
  }
  mutable_unknown_fields()->MergeFrom(from.unknown_fields());
}

void SynchronizedNFrameImu::CopyFrom(const ::google::protobuf::Message& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void SynchronizedNFrameImu::CopyFrom(const SynchronizedNFrameImu& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool SynchronizedNFrameImu::IsInitialized() const {

  return true;
}

void SynchronizedNFrameImu::Swap(SynchronizedNFrameImu* other) {
  if (other != this) {
    imu_timestamps_.Swap(&other->imu_timestamps_);
    imu_measurements_.Swap(&other->imu_measurements_);
    std::swap(nframe_, other->nframe_);
    std::swap(motion_wrt_last_nframe_, other->motion_wrt_last_nframe_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    _unknown_fields_.Swap(&other->_unknown_fields_);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::google::protobuf::Metadata SynchronizedNFrameImu::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = SynchronizedNFrameImu_descriptor_;
  metadata.reflection = SynchronizedNFrameImu_reflection_;
  return metadata;
}


// ===================================================================

#ifndef _MSC_VER
const int ViNodeState::kTWBFieldNumber;
const int ViNodeState::kWVBFieldNumber;
const int ViNodeState::kAccBiasFieldNumber;
const int ViNodeState::kGyroBiasFieldNumber;
#endif  // !_MSC_VER

ViNodeState::ViNodeState()
  : ::google::protobuf::Message() {
  SharedCtor();
  // @@protoc_insertion_point(constructor:vio.proto.ViNodeState)
}

void ViNodeState::InitAsDefaultInstance() {
}

ViNodeState::ViNodeState(const ViNodeState& from)
  : ::google::protobuf::Message() {
  SharedCtor();
  MergeFrom(from);
  // @@protoc_insertion_point(copy_constructor:vio.proto.ViNodeState)
}

void ViNodeState::SharedCtor() {
  _cached_size_ = 0;
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
}

ViNodeState::~ViNodeState() {
  // @@protoc_insertion_point(destructor:vio.proto.ViNodeState)
  SharedDtor();
}

void ViNodeState::SharedDtor() {
  if (this != default_instance_) {
  }
}

void ViNodeState::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* ViNodeState::descriptor() {
  protobuf_AssignDescriptorsOnce();
  return ViNodeState_descriptor_;
}

const ViNodeState& ViNodeState::default_instance() {
  if (default_instance_ == NULL) protobuf_AddDesc_vio_2dcommon_2fvio_5ftypes_2eproto();
  return *default_instance_;
}

ViNodeState* ViNodeState::default_instance_ = NULL;

ViNodeState* ViNodeState::New() const {
  return new ViNodeState;
}

void ViNodeState::Clear() {
  t_w_b_.Clear();
  w_v_b_.Clear();
  acc_bias_.Clear();
  gyro_bias_.Clear();
  ::memset(_has_bits_, 0, sizeof(_has_bits_));
  mutable_unknown_fields()->Clear();
}

bool ViNodeState::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:vio.proto.ViNodeState)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoff(127);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // repeated double T_W_B = 1;
      case 1: {
        if (tag == 9) {
         parse_T_W_B:
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 1, 9, input, this->mutable_t_w_b())));
        } else if (tag == 10) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitiveNoInline<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, this->mutable_t_w_b())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(9)) goto parse_T_W_B;
        if (input->ExpectTag(17)) goto parse_W_v_B;
        break;
      }

      // repeated double W_v_B = 2;
      case 2: {
        if (tag == 17) {
         parse_W_v_B:
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 1, 17, input, this->mutable_w_v_b())));
        } else if (tag == 18) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitiveNoInline<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, this->mutable_w_v_b())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(17)) goto parse_W_v_B;
        if (input->ExpectTag(25)) goto parse_acc_bias;
        break;
      }

      // repeated double acc_bias = 3;
      case 3: {
        if (tag == 25) {
         parse_acc_bias:
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 1, 25, input, this->mutable_acc_bias())));
        } else if (tag == 26) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitiveNoInline<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, this->mutable_acc_bias())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(25)) goto parse_acc_bias;
        if (input->ExpectTag(33)) goto parse_gyro_bias;
        break;
      }

      // repeated double gyro_bias = 4;
      case 4: {
        if (tag == 33) {
         parse_gyro_bias:
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 1, 33, input, this->mutable_gyro_bias())));
        } else if (tag == 34) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitiveNoInline<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, this->mutable_gyro_bias())));
        } else {
          goto handle_unusual;
        }
        if (input->ExpectTag(33)) goto parse_gyro_bias;
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
  // @@protoc_insertion_point(parse_success:vio.proto.ViNodeState)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:vio.proto.ViNodeState)
  return false;
#undef DO_
}

void ViNodeState::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:vio.proto.ViNodeState)
  // repeated double T_W_B = 1;
  for (int i = 0; i < this->t_w_b_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(
      1, this->t_w_b(i), output);
  }

  // repeated double W_v_B = 2;
  for (int i = 0; i < this->w_v_b_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(
      2, this->w_v_b(i), output);
  }

  // repeated double acc_bias = 3;
  for (int i = 0; i < this->acc_bias_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(
      3, this->acc_bias(i), output);
  }

  // repeated double gyro_bias = 4;
  for (int i = 0; i < this->gyro_bias_size(); i++) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(
      4, this->gyro_bias(i), output);
  }

  if (!unknown_fields().empty()) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:vio.proto.ViNodeState)
}

::google::protobuf::uint8* ViNodeState::SerializeWithCachedSizesToArray(
    ::google::protobuf::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:vio.proto.ViNodeState)
  // repeated double T_W_B = 1;
  for (int i = 0; i < this->t_w_b_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteDoubleToArray(1, this->t_w_b(i), target);
  }

  // repeated double W_v_B = 2;
  for (int i = 0; i < this->w_v_b_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteDoubleToArray(2, this->w_v_b(i), target);
  }

  // repeated double acc_bias = 3;
  for (int i = 0; i < this->acc_bias_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteDoubleToArray(3, this->acc_bias(i), target);
  }

  // repeated double gyro_bias = 4;
  for (int i = 0; i < this->gyro_bias_size(); i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      WriteDoubleToArray(4, this->gyro_bias(i), target);
  }

  if (!unknown_fields().empty()) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:vio.proto.ViNodeState)
  return target;
}

int ViNodeState::ByteSize() const {
  int total_size = 0;

  // repeated double T_W_B = 1;
  {
    int data_size = 0;
    data_size = 8 * this->t_w_b_size();
    total_size += 1 * this->t_w_b_size() + data_size;
  }

  // repeated double W_v_B = 2;
  {
    int data_size = 0;
    data_size = 8 * this->w_v_b_size();
    total_size += 1 * this->w_v_b_size() + data_size;
  }

  // repeated double acc_bias = 3;
  {
    int data_size = 0;
    data_size = 8 * this->acc_bias_size();
    total_size += 1 * this->acc_bias_size() + data_size;
  }

  // repeated double gyro_bias = 4;
  {
    int data_size = 0;
    data_size = 8 * this->gyro_bias_size();
    total_size += 1 * this->gyro_bias_size() + data_size;
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

void ViNodeState::MergeFrom(const ::google::protobuf::Message& from) {
  GOOGLE_CHECK_NE(&from, this);
  const ViNodeState* source =
    ::google::protobuf::internal::dynamic_cast_if_available<const ViNodeState*>(
      &from);
  if (source == NULL) {
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
    MergeFrom(*source);
  }
}

void ViNodeState::MergeFrom(const ViNodeState& from) {
  GOOGLE_CHECK_NE(&from, this);
  t_w_b_.MergeFrom(from.t_w_b_);
  w_v_b_.MergeFrom(from.w_v_b_);
  acc_bias_.MergeFrom(from.acc_bias_);
  gyro_bias_.MergeFrom(from.gyro_bias_);
  mutable_unknown_fields()->MergeFrom(from.unknown_fields());
}

void ViNodeState::CopyFrom(const ::google::protobuf::Message& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ViNodeState::CopyFrom(const ViNodeState& from) {
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ViNodeState::IsInitialized() const {

  return true;
}

void ViNodeState::Swap(ViNodeState* other) {
  if (other != this) {
    t_w_b_.Swap(&other->t_w_b_);
    w_v_b_.Swap(&other->w_v_b_);
    acc_bias_.Swap(&other->acc_bias_);
    gyro_bias_.Swap(&other->gyro_bias_);
    std::swap(_has_bits_[0], other->_has_bits_[0]);
    _unknown_fields_.Swap(&other->_unknown_fields_);
    std::swap(_cached_size_, other->_cached_size_);
  }
}

::google::protobuf::Metadata ViNodeState::GetMetadata() const {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::Metadata metadata;
  metadata.descriptor = ViNodeState_descriptor_;
  metadata.reflection = ViNodeState_reflection_;
  return metadata;
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace vio

// @@protoc_insertion_point(global_scope)
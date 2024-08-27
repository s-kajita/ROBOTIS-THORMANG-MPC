
#include "commands_aiding.hpp"

#include "../utils/serialization.h"
#include "../mip_interface.h"

#include <assert.h>


namespace mip {
class Serializer;

namespace C {
struct mip_interface;
} // namespace C

namespace commands_aiding {

using ::mip::insert;
using ::mip::extract;
using namespace ::mip::C;

////////////////////////////////////////////////////////////////////////////////
// Shared Type Definitions
////////////////////////////////////////////////////////////////////////////////

void insert(Serializer& serializer, const Time& self)
{
    insert(serializer, self.timebase);
    
    insert(serializer, self.reserved);
    
    insert(serializer, self.nanoseconds);
    
}
void extract(Serializer& serializer, Time& self)
{
    extract(serializer, self.timebase);
    
    extract(serializer, self.reserved);
    
    extract(serializer, self.nanoseconds);
    
}


////////////////////////////////////////////////////////////////////////////////
// Mip Fields
////////////////////////////////////////////////////////////////////////////////

void insert(Serializer& serializer, const FrameConfig& self)
{
    insert(serializer, self.function);
    
    insert(serializer, self.frame_id);
    
    if( self.function == FunctionSelector::WRITE || self.function == FunctionSelector::READ )
    {
        insert(serializer, self.format);
        
    }
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.tracking_enabled);
        
        for(unsigned int i=0; i < 3; i++)
            insert(serializer, self.translation[i]);
        
        if( self.format == FrameConfig::Format::EULER )
        {
            insert(serializer, self.rotation.euler);
            
        }
        if( self.format == FrameConfig::Format::QUATERNION )
        {
            insert(serializer, self.rotation.quaternion);
            
        }
    }
}
void extract(Serializer& serializer, FrameConfig& self)
{
    extract(serializer, self.function);
    
    extract(serializer, self.frame_id);
    
    if( self.function == FunctionSelector::WRITE || self.function == FunctionSelector::READ )
    {
        extract(serializer, self.format);
        
    }
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.tracking_enabled);
        
        for(unsigned int i=0; i < 3; i++)
            extract(serializer, self.translation[i]);
        
        if( self.format == FrameConfig::Format::EULER )
        {
            extract(serializer, self.rotation.euler);
            
        }
        if( self.format == FrameConfig::Format::QUATERNION )
        {
            extract(serializer, self.rotation.quaternion);
            
        }
    }
}

void insert(Serializer& serializer, const FrameConfig::Response& self)
{
    insert(serializer, self.frame_id);
    
    insert(serializer, self.format);
    
    insert(serializer, self.tracking_enabled);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.translation[i]);
    
    if( self.format == FrameConfig::Format::EULER )
    {
        insert(serializer, self.rotation.euler);
        
    }
    if( self.format == FrameConfig::Format::QUATERNION )
    {
        insert(serializer, self.rotation.quaternion);
        
    }
}
void extract(Serializer& serializer, FrameConfig::Response& self)
{
    extract(serializer, self.frame_id);
    
    extract(serializer, self.format);
    
    extract(serializer, self.tracking_enabled);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.translation[i]);
    
    if( self.format == FrameConfig::Format::EULER )
    {
        extract(serializer, self.rotation.euler);
        
    }
    if( self.format == FrameConfig::Format::QUATERNION )
    {
        extract(serializer, self.rotation.quaternion);
        
    }
}

TypedResult<FrameConfig> writeFrameConfig(C::mip_interface& device, uint8_t frameId, FrameConfig::Format format, bool trackingEnabled, const float* translation, const FrameConfig::Rotation& rotation)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, frameId);
    
    insert(serializer, format);
    
    insert(serializer, trackingEnabled);
    
    assert(translation || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, translation[i]);
    
    if( format == FrameConfig::Format::EULER )
    {
        insert(serializer, rotation.euler);
        
    }
    if( format == FrameConfig::Format::QUATERNION )
    {
        insert(serializer, rotation.quaternion);
        
    }
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_FRAME_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<FrameConfig> readFrameConfig(C::mip_interface& device, uint8_t frameId, FrameConfig::Format format, bool* trackingEnabledOut, float* translationOut, FrameConfig::Rotation* rotationOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    insert(serializer, frameId);
    
    insert(serializer, format);
    
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<FrameConfig> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_FRAME_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_FRAME_CONFIG, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        extract(deserializer, frameId);
        
        extract(deserializer, format);
        
        assert(trackingEnabledOut);
        extract(deserializer, *trackingEnabledOut);
        
        assert(translationOut || (3 == 0));
        for(unsigned int i=0; i < 3; i++)
            extract(deserializer, translationOut[i]);
        
        if( format == FrameConfig::Format::EULER )
        {
            extract(deserializer, rotationOut->euler);
            
        }
        if( format == FrameConfig::Format::QUATERNION )
        {
            extract(deserializer, rotationOut->quaternion);
            
        }
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<FrameConfig> saveFrameConfig(C::mip_interface& device, uint8_t frameId)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    insert(serializer, frameId);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_FRAME_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<FrameConfig> loadFrameConfig(C::mip_interface& device, uint8_t frameId)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    insert(serializer, frameId);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_FRAME_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<FrameConfig> defaultFrameConfig(C::mip_interface& device, uint8_t frameId)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    insert(serializer, frameId);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_FRAME_CONFIG, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const EchoControl& self)
{
    insert(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        insert(serializer, self.mode);
        
    }
}
void extract(Serializer& serializer, EchoControl& self)
{
    extract(serializer, self.function);
    
    if( self.function == FunctionSelector::WRITE )
    {
        extract(serializer, self.mode);
        
    }
}

void insert(Serializer& serializer, const EchoControl::Response& self)
{
    insert(serializer, self.mode);
    
}
void extract(Serializer& serializer, EchoControl::Response& self)
{
    extract(serializer, self.mode);
    
}

TypedResult<EchoControl> writeEchoControl(C::mip_interface& device, EchoControl::Mode mode)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::WRITE);
    insert(serializer, mode);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ECHO_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<EchoControl> readEchoControl(C::mip_interface& device, EchoControl::Mode* modeOut)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::READ);
    assert(serializer.isOk());
    
    uint8_t responseLength = sizeof(buffer);
    TypedResult<EchoControl> result = mip_interface_run_command_with_response(&device, DESCRIPTOR_SET, CMD_ECHO_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer), REPLY_ECHO_CONTROL, buffer, &responseLength);
    
    if( result == MIP_ACK_OK )
    {
        Serializer deserializer(buffer, responseLength);
        
        assert(modeOut);
        extract(deserializer, *modeOut);
        
        if( deserializer.remaining() != 0 )
            result = MIP_STATUS_ERROR;
    }
    return result;
}
TypedResult<EchoControl> saveEchoControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::SAVE);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ECHO_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<EchoControl> loadEchoControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::LOAD);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ECHO_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
TypedResult<EchoControl> defaultEchoControl(C::mip_interface& device)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, FunctionSelector::RESET);
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_ECHO_CONTROL, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const PosEcef& self)
{
    insert(serializer, self.time);
    
    insert(serializer, self.frame_id);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.position[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.uncertainty[i]);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, PosEcef& self)
{
    extract(serializer, self.time);
    
    extract(serializer, self.frame_id);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.position[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.uncertainty[i]);
    
    extract(serializer, self.valid_flags);
    
}

TypedResult<PosEcef> posEcef(C::mip_interface& device, const Time& time, uint8_t frameId, const double* position, const float* uncertainty, PosEcef::ValidFlags validFlags)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, time);
    
    insert(serializer, frameId);
    
    assert(position || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, position[i]);
    
    assert(uncertainty || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, uncertainty[i]);
    
    insert(serializer, validFlags);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_POS_ECEF, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const PosLlh& self)
{
    insert(serializer, self.time);
    
    insert(serializer, self.frame_id);
    
    insert(serializer, self.latitude);
    
    insert(serializer, self.longitude);
    
    insert(serializer, self.height);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.uncertainty[i]);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, PosLlh& self)
{
    extract(serializer, self.time);
    
    extract(serializer, self.frame_id);
    
    extract(serializer, self.latitude);
    
    extract(serializer, self.longitude);
    
    extract(serializer, self.height);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.uncertainty[i]);
    
    extract(serializer, self.valid_flags);
    
}

TypedResult<PosLlh> posLlh(C::mip_interface& device, const Time& time, uint8_t frameId, double latitude, double longitude, double height, const float* uncertainty, PosLlh::ValidFlags validFlags)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, time);
    
    insert(serializer, frameId);
    
    insert(serializer, latitude);
    
    insert(serializer, longitude);
    
    insert(serializer, height);
    
    assert(uncertainty || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, uncertainty[i]);
    
    insert(serializer, validFlags);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_POS_LLH, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const HeightAboveEllipsoid& self)
{
    insert(serializer, self.time);
    
    insert(serializer, self.frame_id);
    
    insert(serializer, self.height);
    
    insert(serializer, self.uncertainty);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, HeightAboveEllipsoid& self)
{
    extract(serializer, self.time);
    
    extract(serializer, self.frame_id);
    
    extract(serializer, self.height);
    
    extract(serializer, self.uncertainty);
    
    extract(serializer, self.valid_flags);
    
}

TypedResult<HeightAboveEllipsoid> heightAboveEllipsoid(C::mip_interface& device, const Time& time, uint8_t frameId, float height, float uncertainty, uint16_t validFlags)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, time);
    
    insert(serializer, frameId);
    
    insert(serializer, height);
    
    insert(serializer, uncertainty);
    
    insert(serializer, validFlags);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HEIGHT_ABOVE_ELLIPSOID, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const VelEcef& self)
{
    insert(serializer, self.time);
    
    insert(serializer, self.frame_id);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.uncertainty[i]);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, VelEcef& self)
{
    extract(serializer, self.time);
    
    extract(serializer, self.frame_id);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.uncertainty[i]);
    
    extract(serializer, self.valid_flags);
    
}

TypedResult<VelEcef> velEcef(C::mip_interface& device, const Time& time, uint8_t frameId, const float* velocity, const float* uncertainty, VelEcef::ValidFlags validFlags)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, time);
    
    insert(serializer, frameId);
    
    assert(velocity || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, velocity[i]);
    
    assert(uncertainty || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, uncertainty[i]);
    
    insert(serializer, validFlags);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEL_ECEF, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const VelNed& self)
{
    insert(serializer, self.time);
    
    insert(serializer, self.frame_id);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.uncertainty[i]);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, VelNed& self)
{
    extract(serializer, self.time);
    
    extract(serializer, self.frame_id);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.uncertainty[i]);
    
    extract(serializer, self.valid_flags);
    
}

TypedResult<VelNed> velNed(C::mip_interface& device, const Time& time, uint8_t frameId, const float* velocity, const float* uncertainty, VelNed::ValidFlags validFlags)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, time);
    
    insert(serializer, frameId);
    
    assert(velocity || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, velocity[i]);
    
    assert(uncertainty || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, uncertainty[i]);
    
    insert(serializer, validFlags);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEL_NED, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const VelBodyFrame& self)
{
    insert(serializer, self.time);
    
    insert(serializer, self.frame_id);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.uncertainty[i]);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, VelBodyFrame& self)
{
    extract(serializer, self.time);
    
    extract(serializer, self.frame_id);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.velocity[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.uncertainty[i]);
    
    extract(serializer, self.valid_flags);
    
}

TypedResult<VelBodyFrame> velBodyFrame(C::mip_interface& device, const Time& time, uint8_t frameId, const float* velocity, const float* uncertainty, VelBodyFrame::ValidFlags validFlags)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, time);
    
    insert(serializer, frameId);
    
    assert(velocity || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, velocity[i]);
    
    assert(uncertainty || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, uncertainty[i]);
    
    insert(serializer, validFlags);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_VEL_BODY_FRAME, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const HeadingTrue& self)
{
    insert(serializer, self.time);
    
    insert(serializer, self.frame_id);
    
    insert(serializer, self.heading);
    
    insert(serializer, self.uncertainty);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, HeadingTrue& self)
{
    extract(serializer, self.time);
    
    extract(serializer, self.frame_id);
    
    extract(serializer, self.heading);
    
    extract(serializer, self.uncertainty);
    
    extract(serializer, self.valid_flags);
    
}

TypedResult<HeadingTrue> headingTrue(C::mip_interface& device, const Time& time, uint8_t frameId, float heading, float uncertainty, uint16_t validFlags)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, time);
    
    insert(serializer, frameId);
    
    insert(serializer, heading);
    
    insert(serializer, uncertainty);
    
    insert(serializer, validFlags);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_HEADING_TRUE, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const MagneticField& self)
{
    insert(serializer, self.time);
    
    insert(serializer, self.frame_id);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.magnetic_field[i]);
    
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, self.uncertainty[i]);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, MagneticField& self)
{
    extract(serializer, self.time);
    
    extract(serializer, self.frame_id);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.magnetic_field[i]);
    
    for(unsigned int i=0; i < 3; i++)
        extract(serializer, self.uncertainty[i]);
    
    extract(serializer, self.valid_flags);
    
}

TypedResult<MagneticField> magneticField(C::mip_interface& device, const Time& time, uint8_t frameId, const float* magneticField, const float* uncertainty, MagneticField::ValidFlags validFlags)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, time);
    
    insert(serializer, frameId);
    
    assert(magneticField || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, magneticField[i]);
    
    assert(uncertainty || (3 == 0));
    for(unsigned int i=0; i < 3; i++)
        insert(serializer, uncertainty[i]);
    
    insert(serializer, validFlags);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_MAGNETIC_FIELD, buffer, (uint8_t)mip_serializer_length(&serializer));
}
void insert(Serializer& serializer, const Pressure& self)
{
    insert(serializer, self.time);
    
    insert(serializer, self.frame_id);
    
    insert(serializer, self.pressure);
    
    insert(serializer, self.uncertainty);
    
    insert(serializer, self.valid_flags);
    
}
void extract(Serializer& serializer, Pressure& self)
{
    extract(serializer, self.time);
    
    extract(serializer, self.frame_id);
    
    extract(serializer, self.pressure);
    
    extract(serializer, self.uncertainty);
    
    extract(serializer, self.valid_flags);
    
}

TypedResult<Pressure> pressure(C::mip_interface& device, const Time& time, uint8_t frameId, float pressure, float uncertainty, uint16_t validFlags)
{
    uint8_t buffer[MIP_FIELD_PAYLOAD_LENGTH_MAX];
    Serializer serializer(buffer, sizeof(buffer));
    
    insert(serializer, time);
    
    insert(serializer, frameId);
    
    insert(serializer, pressure);
    
    insert(serializer, uncertainty);
    
    insert(serializer, validFlags);
    
    assert(serializer.isOk());
    
    return mip_interface_run_command(&device, DESCRIPTOR_SET, CMD_PRESSURE, buffer, (uint8_t)mip_serializer_length(&serializer));
}

} // namespace commands_aiding
} // namespace mip


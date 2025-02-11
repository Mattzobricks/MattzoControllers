#include "MCChannel.h"
#include "log4MC.h"

MCChannel::MCChannel(ChannelType portType, std::string address)
	: _portType{portType}, _address{address} {}

ChannelType MCChannel::GetChannelType()
{
	return _portType;
}

std::string MCChannel::GetAddress()
{
	return _address;
}

int MCChannel::GetAddressAsEspPinNumber()
{
	if (_portType != ChannelType::EspPinChannel) {
		log4MC::error("Trying to retrieve an ESP pin number from a non ESP pin channel.");
	}

	return atoi(_address.c_str());
}

std::string MCChannel::GetParentAddress()
{
	return _parentAddress;
}

void MCChannel::SetParentAddress(const std::string address)
{
	_parentAddress = address;
}
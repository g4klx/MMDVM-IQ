/*
 *   Copyright (C) 2015-2023,2025,2026 by Jonathan Naylor G4KLX
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include "DStarDefines.h"
#include "Conf.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cctype>

const int BUFFER_SIZE = 500;

enum class SECTION {
	NONE,
	GENERAL,
	LOG,
	MQTT,
	HOST,
	MODEM,
	SOAPY,
	MULTI
};

CConf::CConf(const std::string& file) :
m_file(file),
m_daemon(false),
m_logDisplayLevel(0U),
m_logMQTTLevel(0U),
m_mqttHost("127.0.0.1"),
m_mqttPort(1883),
m_mqttKeepalive(60U),
m_mqttName("mmdvm-iq"),
m_mqttAuthEnabled(false),
m_mqttUsername(),
m_mqttPassword(),
m_hostAddress("127.0.0.1"),
m_hostPort(3335U),
m_localAddress("127.0.0.1"),
m_localPort(3334U),
m_trace(false),
m_modemDriver("Soapy"),
m_modemTrace(false),
m_soapyType("sx"),
m_soapyURI(),
m_soapyRXGain(0U),
m_soapyTXGain(0U),
m_multiModemAddress("127.0.0.1"),
m_multiModemPort(48200),
m_multiLocalAddress("127.0.0.1"),
m_multiLocalPort(48100)
{
}

CConf::~CConf()
{
}

bool CConf::read()
{
	FILE* fp = ::fopen(m_file.c_str(), "rt");
	if (fp == nullptr) {
		::fprintf(stderr, "Couldn't open the .ini file - %s\n", m_file.c_str());
		return false;
	}

	SECTION section = SECTION::NONE;

	char buffer[BUFFER_SIZE];
	while (::fgets(buffer, BUFFER_SIZE, fp) != nullptr) {
		if (buffer[0U] == '#')
			continue;

		if (buffer[0U] == '[') {
			if (::strncmp(buffer, "[General]", 9U) == 0)
				section = SECTION::GENERAL;
			else if (::strncmp(buffer, "[Log]", 5U) == 0)
				section = SECTION::LOG;
			else if (::strncmp(buffer, "[MQTT]", 6U) == 0)
				section = SECTION::MQTT;
			else if (::strncmp(buffer, "[Host]", 6U) == 0)
				section = SECTION::HOST;
			else if (::strncmp(buffer, "[Modem]", 7U) == 0)
				section = SECTION::MODEM;
			else if (::strncmp(buffer, "[Soapy]", 7U) == 0)
				section = SECTION::SOAPY;
			else if (::strncmp(buffer, "[Multi]", 7U) == 0)
				section = SECTION::MULTI;
			else
				section = SECTION::NONE;

			continue;
		}

		char* key = ::strtok(buffer, " \t=\r\n");
		if (key == nullptr)
			continue;

		char* value = ::strtok(nullptr, "\r\n");
		if (value == nullptr)
			continue;

		// Remove quotes from the value
		size_t len = ::strlen(value);
		if (len > 1U && *value == '"' && value[len - 1U] == '"') {
			value[len - 1U] = '\0';
			value++;
		} else {
			char *p;

			// if value is not quoted, remove after # (to make comment)
			if ((p = strchr(value, '#')) != nullptr)
				*p = '\0';

			// remove trailing tab/space
			for (p = value + strlen(value) - 1U; p >= value && (*p == '\t' || *p == ' '); p--)
				*p = '\0';
		}

		if (section == SECTION::GENERAL) {
			if (::strcmp(key, "Daemon") == 0)
				m_daemon = ::atoi(value) == 1;
		} else if (section == SECTION::LOG) {
			if (::strcmp(key, "MQTTLevel") == 0)
				m_logMQTTLevel = (unsigned int)::atoi(value);
			else if (::strcmp(key, "DisplayLevel") == 0)
				m_logDisplayLevel = (unsigned int)::atoi(value);
		} else if (section == SECTION::MQTT) {
			if (::strcmp(key, "Host") == 0)
				m_mqttHost = value;
			else if (::strcmp(key, "Port") == 0)
				m_mqttPort = (unsigned short)::atoi(value);
			else if (::strcmp(key, "Keepalive") == 0)
				m_mqttKeepalive = (unsigned int)::atoi(value);
			else if (::strcmp(key, "Name") == 0)
				m_mqttName = value;
			else if (::strcmp(key, "Auth") == 0)
				m_mqttAuthEnabled = ::atoi(value) == 1;
			else if (::strcmp(key, "Username") == 0)
				m_mqttUsername = value;
			else if (::strcmp(key, "Password") == 0)
				m_mqttPassword = value;
		} else if (section == SECTION::HOST) {
			if (::strcmp(key, "HostAddress") == 0)
				m_hostAddress = value;
			else if (::strcmp(key, "HostPort") == 0)
				m_hostPort = (unsigned short)::atoi(value);
			else if (::strcmp(key, "LocalAddress") == 0)
				m_localAddress = value;
			else if (::strcmp(key, "LocalPort") == 0)
				m_localPort = (unsigned short)::atoi(value);
			else if (::strcmp(key, "Trace") == 0)
				m_trace = ::atoi(value) == 1;
		} else if (section == SECTION::MODEM) {
			if (::strcmp(key, "Driver") == 0)
				m_modemDriver = value;
			else if (::strcmp(key, "Trace") == 0)
				m_modemTrace = ::atoi(value) == 1;
		} else if (section == SECTION::SOAPY) {
			if (::strcmp(key, "Type") == 0)
				m_soapyType = value;
			else if (::strcmp(key, "URI") == 0)
				m_soapyURI = value;
			else if (::strcmp(key, "RxGain") == 0)
				m_soapyRXGain = (unsigned int)::atoi(value);
			else if (::strcmp(key, "TxGain") == 0)
				m_soapyTXGain = (unsigned int)::atoi(value);
		} else if (section == SECTION::MULTI) {
			if (::strcmp(key, "ModemAddress") == 0)
				m_multiModemAddress = value;
			else if (::strcmp(key, "ModemPort") == 0)
				m_multiModemPort = (unsigned short)::atoi(value);
			else if (::strcmp(key, "LocalAddress") == 0)
				m_multiLocalAddress = value;
			else if (::strcmp(key, "LocalPort") == 0)
				m_multiLocalPort = (unsigned short)::atoi(value);
		}
	}

	::fclose(fp);

	return true;
}

bool CConf::getDaemon() const
{
	return m_daemon;
}

unsigned int CConf::getLogDisplayLevel() const
{
	return m_logDisplayLevel;
}

unsigned int CConf::getLogMQTTLevel() const
{
	return m_logMQTTLevel;
}

std::string CConf::getMQTTHost() const
{
	return m_mqttHost;
}

unsigned short CConf::getMQTTPort() const
{
	return m_mqttPort;
}

unsigned int CConf::getMQTTKeepalive() const
{
	return m_mqttKeepalive;
}

std::string CConf::getMQTTName() const
{
	return m_mqttName;
}

bool CConf::getMQTTAuthEnabled() const
{
	return m_mqttAuthEnabled;
}

std::string CConf::getMQTTUsername() const
{
	return m_mqttUsername;
}

std::string CConf::getMQTTPassword() const
{
	return m_mqttPassword;
}

std::string CConf::getHostAddress() const
{
	return m_hostAddress;
}

unsigned short CConf::getHostPort() const
{
	return m_hostPort;
}

std::string CConf::getLocalAddress() const
{
	return m_localAddress;
}

unsigned short CConf::getLocalPort() const
{
	return m_localPort;
}

bool CConf::getTrace() const
{
	return m_trace;
}

std::string CConf::getModemDriver() const
{
	return m_modemDriver;
}

bool CConf::getModemTrace() const
{
	return m_modemTrace;
}

std::string CConf::getSoapyType() const
{
	return m_soapyType;
}

std::string CConf::getSoapyURI() const
{
	return m_soapyURI;
}

unsigned int CConf::getSoapyRXGain() const
{
	return m_soapyRXGain;
}

unsigned int CConf::getSoapyTXGain() const
{
	return m_soapyTXGain;
}

std::string CConf::getMultiModemAddress() const
{
	return m_multiModemAddress;
}

unsigned short CConf::getMultiModemPort() const
{
	return m_multiModemPort;
}

std::string CConf::getMultiLocalAddress() const
{
	return m_multiLocalAddress;
}

unsigned short CConf::getMultiLocalPort() const
{
	return m_multiLocalPort;
}

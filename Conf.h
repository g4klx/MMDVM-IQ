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

#if !defined(CONF_H)
#define	CONF_H

#include <string>
#include <cstdint>

class CConf
{
public:
	CConf(const std::string& file);
	~CConf();

	bool read();

	// The General section
	bool         getDaemon() const;

	// The Log section
	unsigned int getLogDisplayLevel() const;
	unsigned int getLogMQTTLevel() const;

    // The MQTT section
    std::string    getMQTTHost() const;
    unsigned short getMQTTPort() const;
    unsigned int   getMQTTKeepalive() const;
    std::string    getMQTTName() const;
    bool           getMQTTAuthEnabled() const;
    std::string    getMQTTUsername() const;
    std::string    getMQTTPassword() const;

	// The Host section
	std::string    getHostAddress() const;
	unsigned short getHostPort() const;
	std::string    getLocalAddress() const;
	unsigned short getLocalPort() const;
	bool           getTrace() const;

	// The Modem section
	std::string  getModemDriver() const;
	bool         getModemTrace() const;

	// The Soapy section
	std::string  getSoapyType() const;
	std::string  getSoapyURI() const;
	unsigned int getSoapyRXGain() const;
	unsigned int getSoapyTXGain() const;

	// The MMDVM-Multi section
	std::string    getMultiModemAddress() const;
	unsigned short getMultiModemPort() const;
	std::string    getMultiLocalAddress() const;
	unsigned short getMultiLocalPort() const;

private:
	std::string m_file;

	bool         m_daemon;

	unsigned int m_logDisplayLevel;
	unsigned int m_logMQTTLevel;

	std::string  m_mqttHost;
	unsigned short m_mqttPort;
	unsigned int m_mqttKeepalive;
	std::string  m_mqttName;
	bool         m_mqttAuthEnabled;
	std::string  m_mqttUsername;
	std::string  m_mqttPassword;

	std::string    m_hostAddress;
	unsigned short m_hostPort;
	std::string    m_localAddress;
	unsigned short m_localPort;
	bool           m_trace;

	std::string  m_modemDriver;
	bool         m_modemTrace;

	std::string  m_soapyType;
	std::string  m_soapyURI;
	unsigned int m_soapyRXGain;
	unsigned int m_soapyTXGain;

	std::string    m_multiModemAddress;
	unsigned short m_multiModemPort;
	std::string    m_multiLocalAddress;
	unsigned short m_multiLocalPort;
};

#endif

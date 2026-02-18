#include "rtl_tcp_client.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <thread>
#include <mutex>

RTLTCPClient::RTLTCPClient(const std::string& host, uint16_t port)
    : m_host(host)
    , m_port(port)
    , m_socket(-1)
    , m_connected(false)
    , m_frequency(0)
    , m_sampleRate(1024000)
    , m_havePendingIqByte(false)
    , m_pendingIqByte(0) {
}

RTLTCPClient::~RTLTCPClient() {
    disconnect();
}

bool RTLTCPClient::connect() {
    struct addrinfo hints;
    std::memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    struct addrinfo* results = nullptr;
    const std::string portStr = std::to_string(m_port);
    const int gai = getaddrinfo(m_host.c_str(), portStr.c_str(), &hints, &results);
    if (gai != 0 || !results) {
        std::cerr << "Invalid address: " << m_host
                  << " (" << gai_strerror(gai) << ")" << std::endl;
        return false;
    }

    m_socket = -1;
    for (struct addrinfo* ai = results; ai != nullptr; ai = ai->ai_next) {
        const int sock = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (sock < 0) {
            continue;
        }
        if (::connect(sock, ai->ai_addr, ai->ai_addrlen) == 0) {
            m_socket = sock;
            break;
        }
        close(sock);
    }
    freeaddrinfo(results);

    if (m_socket < 0) {
        std::cerr << "Failed to connect to " << m_host << ":" << m_port << std::endl;
        return false;
    }

    // Guard initial rtl_tcp header read so connect() cannot block forever.
    struct timeval tv;
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    setsockopt(m_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    uint8_t header[12];
    if (readResponse(header, sizeof(header))) {
        tv.tv_sec = 2;
        tv.tv_usec = 0;
        setsockopt(m_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        m_havePendingIqByte = false;
        m_connected = true;
        return true;
    }

    std::cerr << "Invalid response from rtl_tcp server" << std::endl;
    close(m_socket);
    m_socket = -1;
    return false;
}

void RTLTCPClient::disconnect() {
    if (m_socket >= 0) {
        close(m_socket);
        m_socket = -1;
    }
    m_connected = false;
    m_havePendingIqByte = false;
}

bool RTLTCPClient::sendAll(const uint8_t* data, size_t len) {
    if (!m_connected || m_socket < 0) {
        return false;
    }

    size_t sent = 0;
    while (sent < len) {
        ssize_t n = send(m_socket, data + sent, len - sent, 0);
        if (n <= 0) {
            return false;
        }
        sent += static_cast<size_t>(n);
    }
    return true;
}

bool RTLTCPClient::sendCommand(uint8_t cmd, uint32_t param) {
    if (!m_connected || m_socket < 0) {
        return false;
    }

    uint8_t buffer[5];
    buffer[0] = cmd;
    uint32_t networkOrder = htonl(param);
    memcpy(&buffer[1], &networkOrder, sizeof(networkOrder));
    return sendAll(buffer, sizeof(buffer));
}

bool RTLTCPClient::readResponse(uint8_t* buffer, size_t len) {
    if (m_socket < 0) {
        return false;
    }

    size_t totalRead = 0;
    while (totalRead < len) {
        ssize_t n = recv(m_socket, buffer + totalRead, len - totalRead, 0);
        if (n <= 0) {
            return false;
        }
        totalRead += n;
    }
    return true;
}

size_t RTLTCPClient::readIQ(uint8_t* buffer, size_t maxSamples) {
    if (!m_connected || m_socket < 0) {
        return 0;
    }

    size_t bytesToRead = maxSamples * 2;
    if (bytesToRead == 0) {
        return 0;
    }
    size_t totalRead = 0;
    if (m_havePendingIqByte) {
        buffer[0] = m_pendingIqByte;
        m_havePendingIqByte = false;
        totalRead = 1;
    }

    while (totalRead < bytesToRead) {
        ssize_t n = recv(m_socket, buffer + totalRead, bytesToRead - totalRead, 0);
        if (n > 0) {
            totalRead += static_cast<size_t>(n);
            continue;
        }
        if (n == 0) {
            m_connected = false;
            break;
        }
        if (errno == EINTR) {
            continue;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            break;
        }
        break;
    }

    if ((totalRead & 1u) != 0u) {
        m_pendingIqByte = buffer[totalRead - 1];
        m_havePendingIqByte = true;
        totalRead--;
    }

    return totalRead / 2;
}

bool RTLTCPClient::setFrequency(uint32_t freqHz) {
    if (sendCommand(0x01, freqHz)) {
        m_frequency = freqHz;
        return true;
    }
    return false;
}

bool RTLTCPClient::setSampleRate(uint32_t rate) {
    if (sendCommand(0x02, rate)) {
        m_sampleRate = rate;
        return true;
    }
    return false;
}

bool RTLTCPClient::setGainMode(bool manual) {
    return sendCommand(0x03, manual ? 1u : 0u);
}

bool RTLTCPClient::setGain(uint32_t gain) {
    return sendCommand(0x04, gain);
}

bool RTLTCPClient::setAGC(bool enable) {
    return sendCommand(0x08, enable ? 1u : 0u);
}

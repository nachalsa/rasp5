#!/bin/bash
#sudo vi /usr/local/bin/setup_wipicar_hotspot.ch
#in raspi

# 스크립트 실행 로그를 남기기 위한 함수
log_message() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" | sudo tee -a /var/log/wipicar_hotspot.log
}

log_message "Starting __wipicar hotspot setup script (minimal & robust)."

# --- 설정 변수 ---
WIFI_INTERFACE="wlan0"
HOTSPOT_SSID="__wipicar"
HOTSPOT_PASSWORD="12345678"
CONNECTION_NAME="WipicarHotspotConnection"
IP_ADDRESS="10.42.0.1/24"
SHARED_METHOD="shared" # 'shared'는 DHCP 서버를 포함함을 의미
# --- 설정 변수 끝 ---

# 1. Wi-Fi 인터페이스 유효성 검사
if ! nmcli device show "$WIFI_INTERFACE" &> /dev/null; then
    log_message "ERROR: Wi-Fi interface '$WIFI_INTERFACE' not found or not recognized by NetworkManager."
    log_message "Please ensure the Wi-Fi module is connected and working, and that '$WIFI_INTERFACE' is the correct name."
    exit 1
fi

# 2. 핫스팟 연결 존재 여부 확인
if nmcli connection show | grep -q "$CONNECTION_NAME"; then
    log_message "Hotspot connection '$CONNECTION_NAME' already exists. Ensuring autoconnect is enabled."
    # 연결이 존재하면 autoconnect 속성만 확인하고 설정 변경
    sudo nmcli connection modify "$CONNECTION_NAME" connection.autoconnect yes \
        ipv4.method "$SHARED_METHOD" \
        ipv4.addresses "$IP_ADDRESS" \
        ssid "$HOTSPOT_SSID" \
        mode ap \
        wifi-sec.key-mgmt wpa-psk \
        wifi-sec.psk "$HOTSPOT_PASSWORD" # 보안 설정도 업데이트
    if [ $? -ne 0 ]; then
         log_message "WARNING: Failed to modify existing hotspot connection '$CONNECTION_NAME'."
    fi
else
    # 핫스팟 연결이 존재하지 않으면 새로 생성
    log_message "Hotspot connection '$CONNECTION_NAME' not found. Creating a new one (SSID: $HOTSPOT_SSID)."
    sudo nmcli connection add type wifi ifname "$WIFI_INTERFACE" con-name "$CONNECTION_NAME" \
        ssid "$HOTSPOT_SSID" mode ap wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$HOTSPOT_PASSWORD" \
        ipv4.method "$SHARED_METHOD" \
        ipv4.addresses "$IP_ADDRESS" \
        connection.autoconnect yes

    if [ $? -eq 0 ]; then
        log_message "Hotspot connection '$CONNECTION_NAME' created successfully (SSID: $HOTSPOT_SSID, IP: $IP_ADDRESS)."
    else
        log_message "CRITICAL ERROR: Failed to create hotspot connection '$CONNECTION_NAME'. Exiting."
        exit 1
    fi
fi

log_message "NetworkManager will now handle activation of '$CONNECTION_NAME' (autoconnect enabled)."
log_message "To connect, use SSID: $HOTSPOT_SSID and Password: $HOTSPOT_PASSWORD"
log_message "Clients will get IP addresses in the $IP_ADDRESS range."
log_message "__wipicar hotspot setup script finished successfully."

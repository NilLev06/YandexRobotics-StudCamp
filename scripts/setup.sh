#!/bin/bash
set -e

REPO_DIR="$HOME/robomarvel"
REPO_URL="https://github.com/AndBondStyle/rbm"
SETUP_DIR="$HOME/.setup"
VENV_DIR="$SETUP_DIR/venv"
TEST_SCRIPT_URL="https://raw.githubusercontent.com/AndBondStyle/rbm/refs/heads/master/scripts/test.py"
TEST_SCRIPT_PATH="$SETUP_DIR/test.py"
REQUIREMENTS="pyserial nicegui"
RPI_EEPROM_FREEZE_URL="https://github.com/raspberrypi/rpi-eeprom/raw/refs/heads/master/firmware-2712/old/default/pieeprom-2024-09-10.bin"
RPI_EEPROM_FREEZE_SOURCE="/tmp/pieeprom-2024-09-10.bin"
RPI_EEPROM_FREEZE_CONFIG="/tmp/boot.conf"
RPI_EEPROM_FREEZE_IMAGE="/tmp/pieeprom-freeze.bin"
SUDOERS_FILE="/etc/sudoers.d/010_$USER-nopasswd"
SUDOERS_CONTENT="$USER ALL=(ALL) NOPASSWD:ALL"


NEEDS_REBOOT=false

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

if [ "$EUID" -eq 0 ]; then 
    log_error "Не запускайте скрипт от пользователя root!"
    log_error "Используйте обычного пользователя с правами sudo"
    exit 1
fi

install_docker() {
    log_info "Проверка установки Docker"
    if command -v docker >/dev/null 2>&1; then
        log_info "Docker уже установлен"
        return 0
    fi
    
    log_info "Установка Docker"
    curl -fsSL https://get.docker.com | sh    
    sudo usermod -aG docker $USER

    log_info "Docker успешно установлен"
    log_info "Пользователь $USER добавлен в группу docker"
    log_warn "Для обновления групп запустите: exec sudo su -l \$USER"
}

install_platformio_udev() {
    log_info "Проверка udev правил для PlatformIO"
    UDEV_RULES_FILE="/etc/udev/rules.d/99-platformio-udev.rules"
    UDEV_RULES_URL="https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules"
    
    if [ -f "$UDEV_RULES_FILE" ] && \
       curl -fsSL "$UDEV_RULES_URL" | diff -q "$UDEV_RULES_FILE" - >/dev/null 2>&1; then
        log_info "Актуальные udev правила для PlatformIO уже установлены"
        return 0
    fi
    
    log_info "Загрузка и установка udev правил для PlatformIO"
    curl -fsSL "$UDEV_RULES_URL" | sudo tee "$UDEV_RULES_FILE" > /dev/null
    sudo chown root:root "$UDEV_RULES_FILE"
    sudo chmod 0644 "$UDEV_RULES_FILE"
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    
    log_info "Udev правила для PlatformIO успешно загружены и установлены"
}

install_platformio_tools() {
    log_info "Установка инструментов PlatformIO"
    PIO_DOWNLOAD_URL="https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py"

    if command -v pio >/dev/null 2>&1; then
        log_info "PlatformIO уже установлен"
        return 0
    fi

    curl -fsSL -o $SETUP_DIR/get-platformio.py $PIO_DOWNLOAD_URL
    python3 $SETUP_DIR/get-platformio.py
    sudo ln -s ~/.platformio/penv/bin/pio /usr/local/bin/pio
    pio pkg install --global --tool platformio/tool-stm32flash
}

configure_config_txt() {
    log_info "Проверка конфигурации /boot/firmware/config.txt"    
    CONFIG_FILE="/boot/firmware/config.txt"
    MODIFIED=false
    
    # Проверка и отключение dtparam=audio=on
    if grep -q "^dtparam=audio=on" "$CONFIG_FILE"; then
        log_info "Отключение dtparam=audio=on"
        sudo sed -i 's/^dtparam=audio=on/#dtparam=audio=on/' "$CONFIG_FILE"
        MODIFIED=true
    fi
    
    # Проверка и добавление noaudio к vc4-kms-v3d
    # Ищем строку с vc4-kms-v3d без noaudio
    if grep -q "^dtoverlay=vc4-kms-v3d" "$CONFIG_FILE" && \
       ! grep -q "^dtoverlay=vc4-kms-v3d,noaudio" "$CONFIG_FILE" && \
       ! grep -q "^dtoverlay=vc4-kms-v3d,.*,noaudio" "$CONFIG_FILE"; then
        log_info "Добавление noaudio к vc4-kms-v3d"
        sudo sed -i 's/^dtoverlay=vc4-kms-v3d/dtoverlay=vc4-kms-v3d,noaudio/' "$CONFIG_FILE"
        MODIFIED=true
    fi
    
    # Проверка и добавление dtparam=uart0=on
    if ! grep -q "^dtparam=uart0=on" "$CONFIG_FILE"; then
        log_info "Добавление dtparam=uart0=on"
        echo "dtparam=uart0=on" | sudo tee -a "$CONFIG_FILE" > /dev/null
        MODIFIED=true
    fi
    
    # Проверка и добавление dtparam=i2s=on
    if ! grep -q "^dtparam=i2s=on" "$CONFIG_FILE"; then
        log_info "Добавление dtparam=i2s=on"
        echo "dtparam=i2s=on" | sudo tee -a "$CONFIG_FILE" > /dev/null
        MODIFIED=true
    fi
    
    # Проверка и добавление dtoverlay=googlevoicehat-soundcard
    if ! grep -q "^dtoverlay=googlevoicehat-soundcard" "$CONFIG_FILE"; then
        log_info "Добавление dtoverlay=googlevoicehat-soundcard"
        echo "dtoverlay=googlevoicehat-soundcard" | sudo tee -a "$CONFIG_FILE" > /dev/null
        MODIFIED=true
    fi
    
    if [ "$MODIFIED" = true ]; then
        log_warn "Файл config.txt был изменен"
        NEEDS_REBOOT=true
    else
        log_info "Файл config.txt уже настроен"
    fi
}

configure_eeprom() {
    log_info "Проверка конфигурации EEPROM"

    sudo systemctl disable rpi-eeprom-update.service

    CURRENT_CONFIG=$(sudo rpi-eeprom-config)

    if echo "$CURRENT_CONFIG" | grep -q "^FREEZE_VERSION=1" && \
       echo "$CURRENT_CONFIG" | grep -q "^PSU_MAX_CURRENT=" && \
       vcgencmd bootloader_version 2>/dev/null | grep -q "^2024/09/10"; then
        log_info "EEPROM уже зафиксирован на версии 2024-09-10 и настроен"
        return 0
    fi

    log_warn "EEPROM будет настроен на pieeprom-2024-09-10.bin"

    log_info "Загрузка pieeprom-2024-09-10.bin"
    curl -fsSL "$RPI_EEPROM_FREEZE_URL" -o "$RPI_EEPROM_FREEZE_SOURCE"

    rpi-eeprom-config "$RPI_EEPROM_FREEZE_SOURCE" > "$RPI_EEPROM_FREEZE_CONFIG"

    if ! grep -q "^FREEZE_VERSION=" "$RPI_EEPROM_FREEZE_CONFIG"; then
        echo "FREEZE_VERSION=1" >> "$RPI_EEPROM_FREEZE_CONFIG"
    fi

    if ! grep -q "^PSU_MAX_CURRENT=" "$RPI_EEPROM_FREEZE_CONFIG"; then
        echo "PSU_MAX_CURRENT=5000" >> "$RPI_EEPROM_FREEZE_CONFIG"
    fi

    rpi-eeprom-config \
        --out "$RPI_EEPROM_FREEZE_IMAGE" \
        --config "$RPI_EEPROM_FREEZE_CONFIG" \
        "$RPI_EEPROM_FREEZE_SOURCE"

    sudo rpi-eeprom-update -d -f "$RPI_EEPROM_FREEZE_IMAGE"
    log_warn "Конфигурация EEPROM обновлена"
    NEEDS_REBOOT=true
}

enable_passwordless_sudo() {
    log_info "Настройка passwordless sudo для $USER"

    if [ -f "$SUDOERS_FILE" ]; then
        if sudo cat "$SUDOERS_FILE" 2>/dev/null | grep -q "^$SUDOERS_CONTENT$"; then
            log_info "Passwordless sudo уже настроен для $USER"
            return 0
        fi
    else
        log_info "Создание файла sudoers для passwordless sudo для $USER"
        echo "$SUDOERS_CONTENT" | sudo tee "$SUDOERS_FILE" > /dev/null
        log_info "Passwordless sudo настроен для $USER"
    fi

    sudo chmod 440 "$SUDOERS_FILE"
}

clone_repository() {
    log_info "Проверка репозитория"
    
    if [ -d "$REPO_DIR/.git" ]; then
        log_info "Репо уже склонирован в $REPO_DIR"
        log_info "Обновление репозитория"
        cd "$REPO_DIR"
        git pull
        return 0
    fi
    
    log_info "Клонирование репозитория"
    git clone "$REPO_URL" "$REPO_DIR"
    cd "$REPO_DIR"

    log_info "Добавление флага autostart"
    touch "$REPO_DIR/.autostart"
}

run_docker_compose() {
    log_info "Запуск Docker Compose"
    cd "$REPO_DIR"
    
    log_info "Загрузка Docker образов"
    sudo docker compose pull

    status=0 && [ -d "$REPO_DIR/install" ] || status=$?

    log_info "Запуск Docker контейнера"
    sudo docker compose up -d --no-build

    if [ $status -ne 0 ]; then
        log_warn "Первый запуск контейнера, задержка для завершения сборки"
        sleep 10
    fi

    log_info "Проверка запущенных контейнеров"
    sudo docker compose ps
}

helper_setup_systemd_service() {
    local service_name="$1"
    local unit_content="$2"
    local service_file="/etc/systemd/system/${service_name}.service"
    log_info "Setting up systemd service $service_name"

    if [[ -f "$service_file" ]]; then
        local existing
        existing=$(sudo cat "$service_file" 2>/dev/null)
        if [[ ! "$existing" == "$unit_content" ]]; then
            log_info "Updating existing service file"
            printf "%s" "$unit_content" | sudo tee "$service_file" >/dev/null
            sudo systemctl daemon-reload
            log_info "systemd daemon reloaded"
        fi
    else
        log_info "Creating new service file"
        printf "%s" "$unit_content" | sudo tee "$service_file" >/dev/null
        sudo systemctl daemon-reload
        log_info "systemd daemon reloaded"
    fi

    if ! sudo systemctl is-enabled --quiet "$service_name" 2>/dev/null; then
        log_info "Enabling $service_name to start on boot"
        sudo systemctl enable "$service_name"
    fi

    if ! sudo systemctl is-active --quiet "$service_name"; then
        log_info "Starting $service_name now"
        sudo systemctl start "$service_name"
    fi
}

install_tests_service() {
    mkdir -p "$SETUP_DIR"

    if [[ ! -d "$VENV_DIR" ]]; then
        log_info "Creating virtual environment in $VENV_DIR"
        python3 -m venv "$VENV_DIR"
    else
        log_info "Virtual environment already exists, skipping creation"
    fi

    log_info "Ensuring pip dependencies"
    if [ -n "$PIP_PROXY" ]; then
        log_info 'Using proxy from $PIP_PROXY'
        $VENV_DIR/bin/pip install --quiet --proxy "$PIP_PROXY" $REQUIREMENTS
    else
        $VENV_DIR/bin/pip install --quiet $REQUIREMENTS
    fi

    log_info "Ensuring $TEST_SCRIPT_PATH is up to date"
    if ! curl -fsSL "$TEST_SCRIPT_URL" -o "$TEST_SCRIPT_PATH"; then
        log_error "Failed to download $TEST_SCRIPT_URL"
    fi
    chmod +x "$TEST_SCRIPT_PATH"

    local service_unit="[Unit]
Description=RBM Web Tests
After=docker.service
Requires=docker.service

[Service]
Type=simple
WorkingDirectory=$SETUP_DIR
ExecStart=$VENV_DIR/bin/python $TEST_SCRIPT_PATH
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target"

    helper_setup_systemd_service "rbm-web-tests" "$service_unit"
}

install_web_term_services() {
    local ttyd_binary_url="https://github.com/tsl0922/ttyd/releases/download/1.7.7/ttyd.aarch64"

    if [ -f "$SETUP_DIR/ttyd" ] 2>&1; then
        log_info "ttyd already installed"
    else
        log_info "Downloading ttyd binary"
        curl -fsSL -o "$SETUP_DIR/ttyd" "$ttyd_binary_url"
        chmod +x "$SETUP_DIR/ttyd"
    fi

    local unit1="[Unit]
Description=RBM Web Terminal (host)
After=network.target

[Service]
Type=simple
User=$USER
WorkingDirectory=/home/$USER
ExecStart=$SETUP_DIR/ttyd -W -p 8100 tmux new -A -s webtmux bash
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target"

    helper_setup_systemd_service "rbm-web-term-host" "$unit1"

    local unit2="[Unit]
Description=RBM Web Terminal (docker)
After=network.target

[Service]
Type=simple
User=$USER
WorkingDirectory=/home/$USER
ExecStart=$SETUP_DIR/ttyd -W -p 8200 docker exec -it ros tmux new -A -s main bash
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target"

    helper_setup_systemd_service "rbm-web-term-docker" "$unit2"
}

main() {
    log_info "=== НАЧАЛО НАСТРОЙКИ RASPBERRY ==="

    enable_passwordless_sudo
    install_docker
    install_platformio_udev
    install_platformio_tools
    install_tests_service
    install_web_term_services
    configure_config_txt
    configure_eeprom
    clone_repository
    run_docker_compose
    
    log_info "=== НАСТРОЙКА ЗАВЕРШЕНА ==="
    if [ "$NEEDS_REBOOT" = true ]; then
        log_warn "Запланирована перезагрузка через 1 минуту"
        log_warn "Для отмены перезагрузки: sudo shutdown -c"
        sudo shutdown -r +1
    fi
}

trap 'log_error "Скрипт прерван!"; exit 1' INT TERM

if [ -z "$TMUX" ]; then
    log_warn "Установка пакетов и перезапуск внутри tmux-сессии"
    sudo apt-get -qq update
    sudo apt-get -qq install -y git curl udev tmux
    tmux new -s rbm-setup "bash \"$0\" \"$@\"; bash" && exit
fi
main "$@"

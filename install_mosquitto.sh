#!/usr/bin/env bash
# install_mosquitto.sh
# Ejecutable, con checklist y barra de progreso. Uso: ./install_mosquitto.sh [--dry-run]

set -euo pipefail
IFS=$'\n\t'

DRY_RUN=0
case ${1:-} in
    "") ;;
    --dry-run) DRY_RUN=1 ;;
    *) echo "Uso: $0 [--dry-run]" >&2; exit 2 ;;
esac

TOTAL_STEPS=4
LABELS=(
    "Actualizar índices y instalar curl"
    "Instalar Mosquitto y clientes"
    "Configurar Mosquitto para acceso local"
    "Reiniciar servicio Mosquitto"
)

CMDS=(
    "sudo apt update && sudo apt install -y curl"
    "sudo apt install -y mosquitto mosquitto-clients"
    "printf 'listener 1883 127.0.0.1\\nallow_anonymous true\\n' | sudo tee /etc/mosquitto/conf.d/local.conf > /dev/null"
    "sudo systemctl restart mosquitto"
)

BAR_WIDTH=40

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
BOLD='\033[1m'
RESET='\033[0m'

print_header() {
    echo
    printf "${BOLD}${BLUE}==============================\n"
    printf " Mosquitto installer (script)\n"
    printf "==============================${RESET}\n\n"
}

draw_progress() {
    local idx=$1
    local total=$2
    local label="$3"
    local pct=$(( (idx*100)/total ))
    local filled=$(( (pct*BAR_WIDTH)/100 ))
    local empty=$(( BAR_WIDTH-filled ))
    printf "[%3d%%] [" "$pct"
    for ((i=0;i<filled;i++)); do printf "%b#%b" "$GREEN" "$RESET"; done
    for ((i=0;i<empty;i++)); do printf "%b-%b" "$BLUE" "$RESET"; done
    printf "] %s\n" "$label"
}

print_checklist() {
    local cur=$1
    for idx in $(seq 0 $((TOTAL_STEPS-1))); do
        local mark=" "
        local label_color=""
        if [[ $idx -lt $cur ]]; then
            mark="x"
            label_color="$GREEN"
        elif [[ $idx -eq $cur ]]; then
            mark=">"
            label_color="$YELLOW"
        else
            label_color="$RESET"
        fi
        printf "[%b%s%b] %2d/%d %b%s%b\n" "$BOLD" "$mark" "$RESET" $((idx+1)) $TOTAL_STEPS "$label_color" "${LABELS[idx]}" "$RESET"
    done
}

run_step() {
    local i=$1
    local cmd="${CMDS[i]}"
    local label="${LABELS[i]}"

    print_checklist $i
    draw_progress $i $TOTAL_STEPS "$label"

    if [[ $DRY_RUN -eq 1 ]]; then
        printf "%b[DRY-RUN]%b Ejecutaría: %s\n" "$YELLOW" "$RESET" "$cmd"
        sleep 1
        return 0
    fi

    if bash -euo pipefail -c "$cmd"; then
        printf "%b[OK]%b %s\n" "$GREEN" "$RESET" "$label"
        return 0
    else
        printf "%b[FAIL]%b %s\n" "$RED" "$RESET" "$label"
        return 1
    fi
}

main() {
    print_header
    echo "Modo: $([[ $DRY_RUN -eq 1 ]] && echo 'DRY-RUN' || echo 'EJECUTANDO')"
    echo

    for i in $(seq 0 $((TOTAL_STEPS-1))); do
        if ! run_step $i; then
            echo
            echo "Installation failed on step $((i+1)): ${LABELS[i]}"
            echo "Revisa la salida anterior para detalles."
            exit 2
        fi
        echo
    done

    print_checklist $TOTAL_STEPS
    draw_progress $TOTAL_STEPS $TOTAL_STEPS "Todos los pasos completados"
    echo
    printf "%b¡Listo!%b\n" "$BOLD$GREEN" "$RESET"
}

main
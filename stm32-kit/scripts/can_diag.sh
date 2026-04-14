#!/usr/bin/env bash
set -euo pipefail

iface="${1:-can0}"
stamp="$(date +%Y%m%d-%H%M%S)"
out_dir="${2:-can-diag-${iface}-${stamp}}"

mkdir -p "$out_dir"

run_to_file() {
    local name="$1"
    shift

    {
        printf '$'
        for arg in "$@"; do
            printf ' %q' "$arg"
        done
        printf '\n'
        "$@"
    } > "${out_dir}/${name}.txt" 2>&1 || true
}

run_to_file uname uname -a
run_to_file ip_link ip -details link show "$iface"
run_to_file ip_addr ip addr show "$iface"
run_to_file ip_stats ip -s link show "$iface"
run_to_file dmesg_tail dmesg
run_to_file lsusb lsusb
run_to_file usb_devices bash -lc "ls /dev/serial/by-id 2>/dev/null || true"
run_to_file journal journalctl -k -n 200 --no-pager

if command -v ethtool >/dev/null 2>&1; then
    run_to_file ethtool_drv ethtool -i "$iface"
fi

if command -v candump >/dev/null 2>&1; then
    timeout 3s candump "$iface" > "${out_dir}/candump.txt" 2>&1 || true
fi

if command -v cangen >/dev/null 2>&1; then
    timeout 3s cangen "$iface" -L 8 -I 123 -D 1122334455667788 > "${out_dir}/cangen.txt" 2>&1 || true
fi

printf 'Wrote diagnostics to %s\n' "$out_dir"

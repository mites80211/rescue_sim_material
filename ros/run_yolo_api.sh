#!/usr/bin/env sh
set -eu

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"

usage() {
  cat <<EOF
Usage:
  $0 [--port PORT] [--model MODEL] [--host HOST] [--device DEVICE] [--conf CONF] [--imgsz IMGSZ]
  $0 PORT MODEL
  $0 PORT MODEL [extra args for yolo_api.py]

Examples:
  $0 --port 8001 --model ./yolo.pt
  $0 8002 ./models/traffic.pt
  $0 8003 ./models/person.pt --device cuda:0

Environment fallbacks:
  PORT, MODEL, HOST, DEVICE, CONF, IMGSZ
EOF
}

MODEL="${MODEL:-}"
HOST="${HOST:-0.0.0.0}"
PORT="${PORT:-8000}"
DEVICE="${DEVICE:-}"
CONF="${CONF:-0.25}"
IMGSZ="${IMGSZ:-640}"
POSITIONAL_PORT=""
POSITIONAL_MODEL=""
EXTRA_ARGS=""

append_arg() {
  if [ -z "$EXTRA_ARGS" ]; then
    EXTRA_ARGS="$1"
  else
    EXTRA_ARGS="$EXTRA_ARGS $1"
  fi
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --help|-h)
      usage
      exit 0
      ;;
    --port|-p)
      shift
      [ "$#" -gt 0 ] || { echo "Missing value for --port" >&2; exit 1; }
      PORT="$1"
      ;;
    --model|-m)
      shift
      [ "$#" -gt 0 ] || { echo "Missing value for --model" >&2; exit 1; }
      MODEL="$1"
      ;;
    --host|-H)
      shift
      [ "$#" -gt 0 ] || { echo "Missing value for --host" >&2; exit 1; }
      HOST="$1"
      ;;
    --device|-d)
      shift
      [ "$#" -gt 0 ] || { echo "Missing value for --device" >&2; exit 1; }
      DEVICE="$1"
      ;;
    --conf|-c)
      shift
      [ "$#" -gt 0 ] || { echo "Missing value for --conf" >&2; exit 1; }
      CONF="$1"
      ;;
    --imgsz|-s)
      shift
      [ "$#" -gt 0 ] || { echo "Missing value for --imgsz" >&2; exit 1; }
      IMGSZ="$1"
      ;;
    --)
      shift
      while [ "$#" -gt 0 ]; do
        append_arg "$(printf "%s" "$1" | sed "s/'/'\\\\''/g; s/.*/'&'/")"
        shift
      done
      break
      ;;
    -*)
      append_arg "$(printf "%s" "$1" | sed "s/'/'\\\\''/g; s/.*/'&'/")"
      ;;
    *)
      if [ -z "$POSITIONAL_PORT" ]; then
        POSITIONAL_PORT="$1"
      elif [ -z "$POSITIONAL_MODEL" ]; then
        POSITIONAL_MODEL="$1"
      else
        append_arg "$(printf "%s" "$1" | sed "s/'/'\\\\''/g; s/.*/'&'/")"
      fi
      ;;
  esac
  shift
done

if [ -n "$POSITIONAL_PORT" ]; then
  PORT="$POSITIONAL_PORT"
fi

if [ -n "$POSITIONAL_MODEL" ]; then
  MODEL="$POSITIONAL_MODEL"
fi

if [ -z "$MODEL" ]; then
  if [ -f "$SCRIPT_DIR/yolo.pt" ]; then
    MODEL="$SCRIPT_DIR/yolo.pt"
  else
    MODEL="yolov8n.pt"
  fi
fi

set -- \
  python3 "$SCRIPT_DIR/yolo_api.py" \
  --model "$MODEL" \
  --host "$HOST" \
  --port "$PORT" \
  --conf "$CONF" \
  --imgsz "$IMGSZ"

if [ -n "$DEVICE" ]; then
  set -- "$@" --device "$DEVICE"
fi

if [ -n "$EXTRA_ARGS" ]; then
  eval "set -- \"\$@\" $EXTRA_ARGS"
fi

exec "$@"

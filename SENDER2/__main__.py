import argparse
import logging
import multiprocessing
from pathlib import Path


def main():
    logging.basicConfig(level=logging.INFO,format="%(asctime)s %(levelname)s %(name)s %(message)s")
    parser = argparse.ArgumentParser(description="SENDER2 — separate live mosaics for UAV1 and UAV2")
    sub = parser.add_subparsers(dest="command",required=True)
    ground = sub.add_parser("ground",help="UDP receivers, independent workers and REST API")
    ground.add_argument("--config",default="SENDER2/config.flight.json")
    ground.add_argument("--api-host",default="127.0.0.1")
    ground.add_argument("--api-port",type=int,default=8002)
    from .sender import add_arguments
    add_arguments(sub.add_parser("sender",help="Onboard capture or explicit photo replay"))
    cal = sub.add_parser("calibrate",help="Pinhole camera calibration from chessboards")
    cal.add_argument("--images",required=True)
    cal.add_argument("--cols",type=int,required=True,help="internal corners, horizontal")
    cal.add_argument("--rows",type=int,required=True,help="internal corners, vertical")
    cal.add_argument("--square-mm",type=float,required=True)
    cal.add_argument("--camera-id",required=True)
    cal.add_argument("--calibration-id",required=True)
    cal.add_argument("--output",required=True)
    args = parser.parse_args()
    if args.command == "ground":
        import uvicorn
        from .api import create_app
        from .models import GroundConfig
        from .runtime import GroundRuntime
        config_path = Path(args.config).resolve()
        cfg = GroundConfig.model_validate_json(config_path.read_text(encoding="utf-8"))
        # Paths in config resolve against config directory, independent of shell CWD.
        cfg.output_dir = str((config_path.parent/cfg.output_dir).resolve())
        for uav in cfg.uavs:
            if uav.calibration_file:
                uav.calibration_file = str((config_path.parent/uav.calibration_file).resolve())
        uvicorn.run(create_app(GroundRuntime(cfg)),host=args.api_host,port=args.api_port,workers=1)
    elif args.command == "sender":
        from .sender import run
        run(args)
    elif args.command == "calibrate":
        from .calibrate import run
        run(args)


if __name__ == "__main__":
    multiprocessing.freeze_support()
    main()

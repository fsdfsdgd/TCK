#!/usr/bin/env python3
"""Run one owned test process with an RSS/time guard, recording measured RSS.

The guard aborts the test (failure), never changes SLAM output/parameters.
"""
import argparse
import csv
import os
import signal
import subprocess
import time

parser=argparse.ArgumentParser()
parser.add_argument("--rss-mb",type=float,default=6000)
parser.add_argument("--seconds",type=float,default=1800)
parser.add_argument("--csv",required=True)
parser.add_argument("command",nargs=argparse.REMAINDER)
args=parser.parse_args()
command=args.command[1:] if args.command[:1]==["--"] else args.command
if not command:parser.error("Missing test command")
started=time.monotonic()
process=subprocess.Popen(command,start_new_session=True)
peak=0.0
failure=False
with open(args.csv,"w",newline="") as output:
    writer=csv.writer(output)
    writer.writerow(("elapsed_s","rss_mb"))
    try:
        while process.poll() is None:
            rss=0.0
            try:
                with open("/proc/{}/status".format(process.pid)) as status:
                    for line in status:
                        if line.startswith("VmRSS:"):rss=float(line.split()[1])/1024
            except FileNotFoundError:pass
            elapsed=time.monotonic()-started
            peak=max(peak,rss)
            writer.writerow((round(elapsed,3),round(rss,3)))
            output.flush()
            if rss>args.rss_mb or elapsed>args.seconds:
                failure=True
                print("FAIL resource guard: rss_mb={:.1f} elapsed_s={:.1f}".format(rss,elapsed),flush=True)
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM)
                    try:process.wait(timeout=5)
                    except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL)
                break
            time.sleep(0.5)
    except KeyboardInterrupt:
        os.killpg(process.pid,signal.SIGINT)
        process.wait()
        raise
result=process.wait()
print("RESOURCE peak_rss_mb={:.1f} elapsed_s={:.1f} exit={}".format(
    peak,time.monotonic()-started,result),flush=True)
raise SystemExit(124 if failure else (result if result>=0 else 128-result))

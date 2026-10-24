#!/usr/bin/env python3
"""Guarded entry for bounded read-only clock/TF diagnostics."""
def normalize_output_arguments(arguments):
    result=list(arguments)
    for index,value in enumerate(result[1:],1):
        if value=="--":break
        if value=="--output":result[index]="--record-dir"
        elif value.startswith("--output="):result[index]="--record-dir="+value.split("=",1)[1]
    return result

if __name__ == "__main__":
    import sys
    sys.argv=normalize_output_arguments(sys.argv)
    from storage_entry import ensure_storage_guard
    ensure_storage_guard(__file__)
    from diagnostics.timing_capture import main
    raise SystemExit(main())

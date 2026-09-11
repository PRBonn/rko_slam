with section("format"):
    line_width = 120
    # If an argument group contains more than this many sub-groups (parg or kwarg
    # groups) then force it to a vertical layout.
    max_subgroups_hwrap = 2
    # If a positional argument group contains more than this many arguments, then
    # force it to a vertical layout.
    max_pargs_hwrap = 4
    # If a cmdline positional group consumes more than this many lines without
    # nesting, then invalidate the layout (and nest)
    max_rows_cmdline = 2

with section("parse"):
    additional_commands = {
        "compat_target_sources": {
            "pargs": 1,
            "flags": [],
            "kwargs": {
                "PRIVATE": "*",
                "PUBLIC": "*",
                "FILE_SET": "*",
                "HEADERS": "*",
                "FILES": "*",
                "BASE_DIRS": 1,
            },
        }
    }

The example PC-side HIDAPI example available here:

http://www.signal11.us/oss/hidapi/

uses a report number of 0x1.

This firmware example does not use report numbers.

I could have made the example do this, but I fail to see the rationale for using report numbers.

A patched version of hidtest.cpp that sets the report number to 0x0 (turning report numbers off) is provided in this directory.

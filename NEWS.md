## Release 0.2
__Breaking changes__

* `SizedMemory(x)` now requires that `x` implements `sizeof`

__New features__

* New function `memchr(x::SizedMemory, b::UInt8)`, which finds the first position of `b` in `x`.

## Release 0.1
Initial release

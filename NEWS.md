## Release 0.4.0
__Breaking changes__

* Breaking: `SizedMemory` is now internal

__Other_
_
* Now falls back to generic code if `ENV["JULIA_CPU_TARGET"]` is set.
* Use libc memchr when byteset length is one
* Add more set ops to `ByteSet`

## Release 0.3.2
* Optimise ByteSet
* Optimise `memchr(x, ::Byte)` by calling libc's memchr
* Add generic method `memchr(::Any, x)`, automatically creating `SizedMemory` from x

## Release 0.3.1
Fix issue #3 - SIMD fallback warning only emitted at package precompile time.

## Release 0.3
__Breaking changes__

* `gen_scan_function` has been removed. Instead, you should now scan for multiple bytes by calling `memchr(::SizedMemory, Val(byteset))`.

## Release 0.2
__Breaking changes__

* `SizedMemory(x)` now requires that `x` implements `sizeof`

__New features__

* New function `memchr(x::SizedMemory, b::UInt8)`, which finds the first position of `b` in `x`.

## Release 0.1
Initial release

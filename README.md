# ScanByte.jl

![CI](https://github.com/jakobnissen/ScanByte.jl/workflows/CI/badge.svg)
[![Codecov](https://codecov.io/gh/jakobnissen/ScanByte.jl/branch/master/graph/badge.svg)](https://codecov.io/gh/jakobnissen/ScanByte.jl)

_Find your bytes. Fast._

ScanByte is a package to solve a simple problem: Find the first occurrence of a byte or set of bytes in a chunk of memory. Think of it like a much faster version of `findfirst` that only iterates over bytes in memory.

ScanByte is micro-optimized for speed. On my laptop it can hit the RAM bandwidth limit of around 20 GB/s. This speed makes it a suitable building block for string search engines, Regex implementations, parsers and similar use cases.

### Usage
The memory you are scanning should be represented by a pointer pointing to the beginning of the memory you want to scan, and a length of the memory.

Alternatively, to avoid messing around with pointers directly, you can use the exported `SizedMemory` object. That's just a struct containing a pointer and a length. You can construct a `SizedMemory`from `String`s, `SubString{String}`s, and anything that implements the functions `pointer` and `sizeof`:
```julia
julia> v = [0x01, 0x02, 0x03];

julia> SizedMemory(v)
SizedMemory(Ptr{UInt8} @0x000000010e2d6e70, 0x0000000000000003)

julia> SizedMemory("ScanByte")
SizedMemory(Ptr{UInt8} @0x000000010f164218, 0x0000000000000008)
```

The central function of interest in this package is `memchr`. To search for a single byte, simply pass the `SizedMemory` (or a pointer and the length) to the function with the byte you are looking for:
```julia
julia> v = [0xda, 0x00, 0x43, 0xf0];

julia> findfirst(isequal(0x43), v) === memchr(SizedMemory(v), 0x43)
true

julia> memchr(SizedMemory(v), 0x43) === memchr(pointer(v), UInt(length(v)), 0x43)
true

julia> findfirst(isequal(0xff), v) === memchr(SizedMemory(v), 0xff)
true
```

If you want to scan for bytes fulfilling an arbitrary predicate `f` (like `findfirst` can do), this is only possible by moving some of the computation to compile time. To do this, first you must represent the set of bytes you are searching for as a `ByteSet` object:

```julia
julia> byteset = ByteSet(filter(f, 0x00:0xff));
```

Then, you pass this byteset as `Val(byteset)`. This will create a function specialized for that particular byteset.

```julia

julia> memchr(SizedMemory(my_vector), Val(byteset))
410068
```

As usual when dealing with `Val`, be mindful that this requires the compilation of a new method instance every time you pass a previously unseen `Val` to the function.

## Drawbacks
At the moment, ScanByte has three major drawbacks:

* It relies on generated functions to compute the optimal Julia code to create the scanning function. This means the byte set must be known at compile time - unless you're scanning for just a single byte.
* It relies on explicit SIMD instructions. To be fast, it must run on computers with ideally the `AVX2` instruction set, or with the `SSE2` and `SSSE3` sets. Also, if you create the scanning function on a computer with `AVX2` but runs it on a computer without, LLVM will probably crash. Currently, the fallback methods are fairly slow.
* There is no guaranteed stable version of detecting which SIMD instructions your Julia supports. So this package tries to guess by parsing some output from LLVM.

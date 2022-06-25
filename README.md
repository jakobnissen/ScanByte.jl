# ScanByte.jl

![CI](https://github.com/jakobnissen/ScanByte.jl/workflows/CI/badge.svg)
[![Codecov](https://codecov.io/gh/jakobnissen/ScanByte.jl/branch/master/graph/badge.svg)](https://codecov.io/gh/jakobnissen/ScanByte.jl)

_Find your bytes. Fast._

ScanByte is a package to solve a simple problem: Find the first occurrence of a byte or any byte from a set of bytes in a chunk of memory. Think of it like a much faster version of `findfirst` that only iterates over bytes in memory.

ScanByte is micro-optimized for speed. On my laptop it can hit the RAM bandwidth limit of around 20 GB/s. This speed makes it a suitable building block for string search engines, Regex implementations, parsers and similar use cases.

### Usage
The central function of interest in this package is `memchr`.
The function takes a chunk of memory and some bytes, and returns the first position (1-indexed) in the chunk of memory where any byte from the byte set is found, or `nothing` if no bytes are found.

The chunk of memory can be any type which implements methods for `pointer` and `sizeof`, or alternatively you can input a raw pointer and a size.

The byte set can be passed in in two different ways:
* As a single `UInt8`, in which case ScanByte will simply dispatch to libc's memchr function
* A `Val{bs}`, where `bs` is an instance of the type `ByteSet <: AbstractSet{UInt8}` from this package.

In the latter case, ScanByte will, at compile time, pick an efficient SIMD algorithm based on the content of the byteset.
Currently ScanByte only has SIMD algorithms for SSSE3 and AVX2 instruction sets (found in all x86-based PCs), and uses a slow fallback for CPUs without these instructions.

### Example usage
In all these examples, the input data can be a `String`, a `codeunits` object, an `Array{UInt8}`, or a pointer+length.
Any type that implements `pointer` and `sizeof` will do.

Search for a single byte:
```julia
julia> memchr("Julia", UInt8('i'))
4

julia> memchr(codeunits("Julia"), UInt8('z')) === nothing
true

julia> str = "Julia";

julia> GC.@preserve str memchr(pointer(str), sizeof(str) % UInt, UInt8('i'))
4
```

Search for a byteset. Here, `Val` must be used to force specialization on the byteset:
```julia
julia> bs = ByteSet([0x01, 0x6a, 0xf1]);

julia> memchr([0x4a, 0xf1], Val(bs))
3
```

Search using a function. To do this, you must construct a `ByteSet` using the predicate on `0x00:0xff`:
```julia
julia> f(x) = in(x, 0x1a:0x4c) || in(x, 0xd1:0xf1); # some function

julia> bs = ByteSet(filter(f, 0x00:0xff));

julia> memchr("hello, Bob", Val(bs))
6
```

## Drawbacks
At the moment, ScanByte has three major drawbacks:

* If you are search for a predicate/byteset and not a single byte, tt relies on generated functions to compute the optimal Julia code to create the scanning function. This means the byte set must be known at compile time.
* It relies on explicit SIMD instructions. To be fast, it must run on computers with ideally the `AVX2` instruction set, or with the `SSE2` and `SSSE3` sets. Also, if you create the scanning function on a computer with `AVX2` but runs it on a computer without, LLVM will probably crash. Currently, the fallback methods are fairly slow.
* There is no guaranteed stable version of detecting which SIMD instructions your Julia supports. So this package tries to guess by parsing some output from LLVM.

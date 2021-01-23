# ScanByte.jl

![CI](https://github.com/jakobnissen/ScanByte.jl/workflows/CI/badge.svg)
[![Codecov](https://codecov.io/gh/jakobnissen/ScanByte.jl/branch/master/graph/badge.svg)](https://codecov.io/gh/jakobnissen/ScanByte.jl)

_Find your bytes. Fast._

ScanByte is a package to solve a simple problem: Find the first byte in a byte array that is member of a compile-time constant set of bytes. It is somewhat similar to `memchr`.

ScanByte is micro-optimized for speed. On my laptop it can hit the RAM bandwidth limit of around 20 GB/s, depending on the input byte set. This speed makes it a suitable building block for string search engines, Regex implementations, parsers and similar use cases.

### Usage
To use it, you first have to specify the set of bytes you are scanning for:

```julia
julia> byteset = ByteSet("Jill")
ByteSet with 3 elements:
  0x4a
  0x69
  0x6c
```

Then, you pass the byteset as well as a `Symbol` to the `gen_scan_function` function. This function returns Julia code that evaluates to a method definition of a function named by your symbol:

```julia
julia> eval(gen_scan_function(:scanobj, byteset))
scanobj (generic function with 1 method)
```

Now, you can use the `scanobj` function with `SizedMemory` object. A `SizedMemory` object contains a pointer and a length, and can be constucted from a `String` or an `Array{UInt8}` (or anything else that provides a pointer and a length.):

```julia
julia> scanobj(SizedMemory([0x03, 0x05, 0x6c, 0xa1]))
0x0000000000000003
```

The function returns the 1-based index of the first byte in the byte set, or `nothing` if no bytes were found. 

At the moment, ScanByte has three major drawbacks:

* It relies on metaprogramming to compute the optimal Julia code to create the scanning function. This means ScanByte might add significantly to compile times, especially when creating tonnes of scanning functions. Furthermore, it means the byte set must be known at compile time.
* It relies on explicit SIMD instructions. It must run on computers with either the `AVX2` instruction set, or with the `SSE2` and `SSSE3` sets. Also, if you create the scanning function on a computer with `AVX2` but runs it on a computer without, LLVM will probably crash.
* There is no guaranteed stable version of detecting which SIMD instructions your Julia supports. So this package tries to guess.

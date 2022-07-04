module ScanByte

using Libdl: Libdl
using SIMD: SIMD

include("byteset.jl")
include("codegen.jl")

export ByteSet, memchr

end # module

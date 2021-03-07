module ScanByte

using Libdl
using SIMD

include("byteset.jl")
include("codegen.jl")

export SizedMemory, ByteSet, memchr

end # module
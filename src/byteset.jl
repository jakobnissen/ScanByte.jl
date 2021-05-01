struct ByteSet <: AbstractSet{UInt8}
    data::NTuple{4, UInt64}
    ByteSet(x::NTuple{4, UInt64}) = new(x)
end

ByteSet() = ByteSet((UInt64(0), UInt64(0), UInt64(0), UInt64(0)))
Base.length(s::ByteSet) = mapreduce(count_ones, +, s.data)
Base.isempty(s::ByteSet) = s === ByteSet()

function ByteSet(it)
    a = b = c = d = UInt64(0)
    for i in it
        vi = convert(UInt8, i)
        if vi < 0x40
            a |= UInt(1) << ((vi - 0x00) & 0x3f)
        elseif vi < 0x80
            b |= UInt(1) << ((vi - 0x40) & 0x3f)
        elseif vi < 0xc0
            c |= UInt(1) << ((vi - 0x80) & 0x3f)
        else
            d |= UInt(1) << ((vi - 0xc0) & 0x3f)
        end
    end
    ByteSet((a, b, c, d))
end

function Base.minimum(s::ByteSet)
    isempty(s) && Base._empty_reduce_error()
    (iterate(s)::Tuple{UInt8, Any})[1]
end

function Base.maximum(s::ByteSet)
    isempty(s) && Base._empty_reduce_error()
    offset = 0x03 * UInt8(64)
    for i in 0:3
        @inbounds bits = s.data[4 - i]
        lz = leading_zeros(bits)
        if lz != 64
            return offset + UInt8(64) - UInt8(lz) - 0x01
        end
        offset -= UInt8(64)
    end
end

function Base.in(byte::UInt8, s::ByteSet)
    i, o = divrem(byte, UInt8(64))
    @inbounds !(iszero(s.data[i & 0x03 + 0x01] >>> (o & 0x3f) & UInt(1)))
end

function Base.iterate(s::ByteSet, state=UInt(0))
    ioffset, offset = divrem(state, UInt(64))
    ioffset == 4 && return nothing
    bits = @inbounds s.data[ioffset & 0x03 + 0x01] >>> offset
    tz = trailing_zeros(bits)
    while tz == 64
        ioffset += 1
        offset = UInt(0)
        ioffset == 4 && return nothing
        bits = s.data[ioffset & 0x03 + 0x01]
        tz = trailing_zeros(bits)
    end
    result = (64 * ioffset + offset + tz) % UInt8
    result, UInt(result) + UInt(1)
end

function Base.:~(s::ByteSet)
    a, b, c, d = s.data
    ByteSet((~a, ~b, ~c, ~d))
end

is_contiguous(s::ByteSet) = isempty(s) || (maximum(s)::UInt8 - minimum(s)::UInt8 + 1 == length(s))
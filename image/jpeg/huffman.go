// Copyright 2009 The Go Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package jpeg

import (
	"io"
)

// maxCodeLength is the maximum (inclusive) number of bits in a Huffman code.
const maxCodeLength = 16

// maxNCodes is the maximum (inclusive) number of codes in a Huffman tree.
const maxNCodes = 256

// lutSize is the log-2 size of the Huffman decoder's look-up table.
const lutSize = 8

// Huffman is a Huffman decoder, specified in section C.
type Huffman struct {
	// length is the number of codes in the tree.
	NCodes int32
	// Lut is the look-up table for the next lutSize bits in the bit-stream.
	// The high 8 bits of the uint16 are the encoded Value. The low 8 bits
	// are 1 plus the code length, or 0 if the Value is too large to fit in
	// lutSize bits.
	Lut [1 << lutSize]uint16

	// Count[i] is the number of codes of length i+1 bits.
	Count [16]uint8

	// Vals are the decoded values, sorted by their encoding.
	Vals [maxNCodes]uint8
	// MinCodes[i] is the minimum code of length i, or -1 if there are no
	// codes of that length.
	MinCodes [maxCodeLength]int32
	// MaxCodes[i] is the maximum code of length i, or -1 if there are no
	// codes of that length.
	MaxCodes [maxCodeLength]int32
	// ValsIndices[i] is the index into vals of MinCodes[i].
	ValsIndices [maxCodeLength]int32
}

// errShortHuffmanData means that an unexpected EOF occurred while decoding
// Huffman data.
var errShortHuffmanData = FormatError("short Huffman data")

// ensureNBits reads bytes from the byte buffer to ensure that d.bits.n is at
// least n. For best performance (avoiding function calls inside hot loops),
// the caller is the one responsible for first checking that d.bits.n < n.
func (d *decoder) ensureNBits(n int32) error {
	for {
		c, err := d.readByteStuffedByte()
		if err != nil {
			if err == io.ErrUnexpectedEOF {
				return errShortHuffmanData
			}
			return err
		}
		d.bits.a = d.bits.a<<8 | uint32(c)
		d.bits.n += 8
		if d.bits.m == 0 {
			d.bits.m = 1 << 7
		} else {
			d.bits.m <<= 8
		}
		if d.bits.n >= n {
			break
		}
	}
	return nil
}

// receiveExtend is the composition of RECEIVE and EXTEND, specified in section
// F.2.2.1.
// t <= 16
func (d *decoder) receiveExtend(t uint8) (int32, int32, error) {
	if d.bits.n < int32(t) {
		if err := d.ensureNBits(int32(t)); err != nil {
			return 0, 0, err
		}
	}
	d.bits.n -= int32(t)
	d.bits.m >>= t
	s := int32(1) << t
	x := int32(d.bits.a>>uint8(d.bits.n)) & (s - 1) // positive
	x0 := x
	if x < s>>1 {
		//x := (1 << t) - 1 - x //  positive
		//x = -x  // negative

		x += int32((-1)<<t) + 1 // negative
	}
	return x, x0, nil
}

// processDHT processes a Define Huffman Table marker, and initializes a Huffman
// struct from its contents. Specified in section B.2.4.2.
func (d *decoder) processDHT(n int) error {
	d.aux.DHTStart = d.bytes.readBytes
	d.aux.DHTN = n
	defer func() {
		d.aux.DHTLen = d.bytes.readBytes - d.aux.DHTStart
	}()
	for n > 0 {
		if n < 17 {
			return FormatError("DHT has wrong length")
		}
		if err := d.readFull(d.tmp[:17]); err != nil {
			return err
		}
		tc := d.tmp[0] >> 4
		if tc > maxTc {
			return FormatError("bad Tc Value")
		}
		th := d.tmp[0] & 0x0f
		// The baseline th <= 1 restriction is specified in table B.5.
		if th > maxTh || (d.baseline && th > 1) {
			return FormatError("bad Th Value")
		}
		h := &d.huff[tc][th]

		// Read NCodes and H.Vals (and derive H.NCodes).
		// NCodes[i] is the number of codes with code length i.
		// H.NCodes is the total number of codes.
		h.NCodes = 0
		var nCodes [maxCodeLength]int32
		for i := range nCodes {
			nCodes[i] = int32(d.tmp[i+1])
			h.NCodes += nCodes[i]
		}
		copy(h.Count[:], d.tmp[1:17])

		if h.NCodes == 0 {
			return FormatError("Huffman table has zero length")
		}
		if h.NCodes > maxNCodes {
			return FormatError("Huffman table has excessive length")
		}
		n -= int(h.NCodes) + 17
		if n < 0 {
			return FormatError("DHT has wrong length")
		}
		if err := d.readFull(h.Vals[:h.NCodes]); err != nil {
			return err
		}

		// Derive the look-up table.
		clear(h.Lut[:])
		var x, code uint32
		for i := uint32(0); i < lutSize; i++ {
			code <<= 1
			for j := int32(0); j < nCodes[i]; j++ {
				// The codeLength is 1+i, so shift code by 8-(1+i) to
				// calculate the high bits for every 8-bit sequence
				// whose codeLength's high bits matches code.
				// The high 8 bits of lutValue are the encoded Value.
				// The low 8 bits are 1 plus the codeLength.
				base := uint8(code << (7 - i))
				lutValue := uint16(h.Vals[x])<<8 | uint16(2+i)
				for k := uint8(0); k < 1<<(7-i); k++ {
					h.Lut[base|k] = lutValue
				}
				code++
				x++
			}
		}

		// Derive MinCodes, MaxCodes, and ValsIndices.
		var c, index int32
		for i, n := range nCodes {
			if n == 0 {
				h.MinCodes[i] = -1
				h.MaxCodes[i] = -1
				h.ValsIndices[i] = -1
			} else {
				h.MinCodes[i] = c
				h.MaxCodes[i] = c + n - 1
				h.ValsIndices[i] = index
				c += n
				index += n
			}
			c <<= 1
		}
	}
	return nil
}

// decodeHuffman returns the next Huffman-coded Value from the bit-stream,
// decoded according to H.
func (d *decoder) decodeHuffman(h *Huffman) (uint8, uint16, uint8, error) {
	if h.NCodes == 0 {
		return 0, 0, 0, FormatError("uninitialized Huffman table")
	}

	if d.bits.n < 8 {
		if err := d.ensureNBits(8); err != nil {
			if err != errMissingFF00 && err != errShortHuffmanData {
				return 0, 0, 0, err
			}
			// There are no more bytes of data in this segment, but we may still
			// be able to read the next symbol out of the previously read bits.
			// First, undo the readByte that the ensureNBits call made.
			if d.bytes.nUnreadable != 0 {
				d.unreadByteStuffedByte()
			}
			goto slowPath
		}
	}
	//if v := h.Lut[(d.bits.a>>uint32(d.bits.n-lutSize))&0xff]; v != 0 {
	//	code := (d.bits.a >> uint32(d.bits.n-lutSize)) & 0xff
	//	n := (v & 0xff) - 1
	//	d.bits.n -= int32(n)
	//	d.bits.m >>= n
	//	return uint8(v >> 8), uint16(code), uint8(n), nil
	//}

slowPath:
	for i, code := 0, int32(0); i < maxCodeLength; i++ {
		if d.bits.n == 0 {
			if err := d.ensureNBits(1); err != nil {
				return 0, 0, 0, err
			}
		}
		if d.bits.a&d.bits.m != 0 {
			code |= 1
		}
		d.bits.n--
		d.bits.m >>= 1
		if code <= h.MaxCodes[i] {
			return h.Vals[h.ValsIndices[i]+code-h.MinCodes[i]], uint16(code), uint8(i + 1), nil
		}
		code <<= 1
	}
	return 0, 0, 0, FormatError("bad Huffman code")
}

func (d *decoder) decodeBit() (bool, error) {
	if d.bits.n == 0 {
		if err := d.ensureNBits(1); err != nil {
			return false, err
		}
	}
	ret := d.bits.a&d.bits.m != 0
	d.bits.n--
	d.bits.m >>= 1
	return ret, nil
}

func (d *decoder) decodeBits(n int32) (uint32, error) {
	if d.bits.n < n {
		if err := d.ensureNBits(n); err != nil {
			return 0, err
		}
	}
	ret := d.bits.a >> uint32(d.bits.n-n)
	ret &= (1 << uint32(n)) - 1
	d.bits.n -= n
	d.bits.m >>= uint32(n)
	return ret, nil
}

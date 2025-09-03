// Copyright 2012 The Go Authors. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

package jpeg

import (
	"image"
)

var gg = -1

const (
	GG = 47
)

// makeImg allocates and initializes the destination image.
func (d *decoder) makeImg(mxx, myy int) {
	if d.nComp == 1 {
		m := image.NewGray(image.Rect(0, 0, 8*mxx, 8*myy))
		d.img1 = m.SubImage(image.Rect(0, 0, d.width, d.height)).(*image.Gray)
		return
	}

	h0 := d.comp[0].H
	v0 := d.comp[0].V
	hRatio := h0 / d.comp[1].H
	vRatio := v0 / d.comp[1].V
	var subsampleRatio image.YCbCrSubsampleRatio
	switch hRatio<<4 | vRatio {
	case 0x11:
		subsampleRatio = image.YCbCrSubsampleRatio444
	case 0x12:
		subsampleRatio = image.YCbCrSubsampleRatio440
	case 0x21:
		subsampleRatio = image.YCbCrSubsampleRatio422
	case 0x22:
		subsampleRatio = image.YCbCrSubsampleRatio420
	case 0x41:
		subsampleRatio = image.YCbCrSubsampleRatio411
	case 0x42:
		subsampleRatio = image.YCbCrSubsampleRatio410
	default:
		panic("unreachable")
	}
	m := image.NewYCbCr(image.Rect(0, 0, 8*h0*mxx, 8*v0*myy), subsampleRatio)
	d.img3 = m.SubImage(image.Rect(0, 0, d.width, d.height)).(*image.YCbCr)

	d.aux.ExtImg3 = m

	if d.nComp == 4 {
		h3, v3 := d.comp[3].H, d.comp[3].V
		d.blackPix = make([]byte, 8*h3*mxx*8*v3*myy)
		d.blackStride = 8 * h3 * mxx
	}
}

// Specified in section B.2.3.
func (d *decoder) processSOS(n int) error {
	d.aux.SOSStart = d.bytes.readBytes
	d.aux.SOSN = n
	defer func() {
		d.aux.SOSLen = d.bytes.readBytes - d.aux.SOSStart
	}()
	if d.nComp == 0 {
		return FormatError("missing SOF marker")
	}
	if n < 6 || 4+2*d.nComp < n || n%2 != 0 {
		return FormatError("SOS has wrong length")
	}
	if err := d.readFull(d.tmp[:n]); err != nil {
		return err
	}
	nComp := int(d.tmp[0])
	if n != 4+2*nComp {
		return FormatError("SOS length inconsistent with number of components")
	}
	var scan [maxComponents]Scan
	totalHV := 0
	for i := 0; i < nComp; i++ {
		cs := d.tmp[1+2*i] // Component selector.
		compIndex := -1
		for j, comp := range d.comp[:d.nComp] {
			if cs == comp.C {
				compIndex = j
			}
		}
		if compIndex < 0 {
			return FormatError("unknown Component selector")
		}
		scan[i].CompIndex = uint8(compIndex)
		// Section B.2.3 states that "the Value of Cs_j shall be different from
		// the values of Cs_1 through Cs_(j-1)". Since we have previously
		// verified that a frame's Component identifiers (C_i values in section
		// B.2.2) are unique, it suffices to check that the implicit indexes
		// into d.comp are unique.
		for j := 0; j < i; j++ {
			if scan[i].CompIndex == scan[j].CompIndex {
				return FormatError("repeated Component selector")
			}
		}
		totalHV += d.comp[compIndex].H * d.comp[compIndex].V

		// The baseline t <= 1 restriction is specified in table B.3.
		scan[i].Td = d.tmp[2+2*i] >> 4
		if t := scan[i].Td; t > maxTh || (d.baseline && t > 1) {
			return FormatError("bad Td Value")
		}
		scan[i].Ta = d.tmp[2+2*i] & 0x0f
		if t := scan[i].Ta; t > maxTh || (d.baseline && t > 1) {
			return FormatError("bad Ta Value")
		}
	}
	d.aux.Scans = scan
	// Section B.2.3 states that if there is more than one Component then the
	// total H*V values in a scan must be <= 10.
	if d.nComp > 1 && totalHV > 10 {
		return FormatError("total sampling factors too large")
	}

	// zigStart and zigEnd are the spectral selection bounds.
	// ah and al are the successive approximation high and low values.
	// The spec calls these values Ss, Se, Ah and Al.
	//
	// For progressive JPEGs, these are the two more-or-less independent
	// aspects of progression. Spectral selection progression is when not
	// all of a Block's 64 DCT coefficients are transmitted in one pass.
	// For example, three passes could transmit coefficient 0 (the DC
	// Component), coefficients 1-5, and coefficients 6-63, in zig-zag
	// order. Successive approximation is when not all of the bits of a
	// band of coefficients are transmitted in one pass. For example,
	// three passes could transmit the 6 most significant bits, followed
	// by the second-least significant bit, followed by the least
	// significant bit.
	//
	// For sequential JPEGs, these parameters are hard-coded to 0/63/0/0, as
	// per table B.3.
	zigStart, zigEnd, ah, al := int32(0), int32(BlockSize-1), uint32(0), uint32(0)
	if d.progressive {
		zigStart = int32(d.tmp[1+2*nComp])
		zigEnd = int32(d.tmp[2+2*nComp])
		ah = uint32(d.tmp[3+2*nComp] >> 4)
		al = uint32(d.tmp[3+2*nComp] & 0x0f)
		if (zigStart == 0 && zigEnd != 0) || zigStart > zigEnd || BlockSize <= zigEnd {
			return FormatError("bad spectral selection bounds")
		}
		if zigStart != 0 && nComp != 1 {
			return FormatError("progressive AC coefficients for more than one Component")
		}
		if ah != 0 && ah != al+1 {
			return FormatError("bad successive approximation values")
		}
	}

	// mxx and myy are the number of MCUs (Minimum Coded Units) in the image.
	h0, v0 := d.comp[0].H, d.comp[0].V // The H and V values from the Y components.
	mxx := (d.width + 8*h0 - 1) / (8 * h0)
	myy := (d.height + 8*v0 - 1) / (8 * v0)
	if d.img1 == nil && d.img3 == nil {
		d.makeImg(mxx, myy)
	}
	if d.progressive {
		for i := 0; i < nComp; i++ {
			compIndex := scan[i].CompIndex
			if d.progCoeffs[compIndex] == nil {
				d.progCoeffs[compIndex] = make([]Block, mxx*myy*d.comp[compIndex].H*d.comp[compIndex].V)
			}
		}
	}

	d.bits = bits{}
	mcu, expectedRST := 0, uint8(rst0Marker)
	var (
		// b is the decoded coefficients, in natural (not zig-zag) order.
		b  Block
		dc [maxComponents]int32
		// bx and by are the location of the current Block, in units of 8x8
		// blocks: the third Block in the first row has (bx, by) = (2, 0).
		bx, by     int
		blockCount int
	)
	for my := 0; my < myy; my++ {
		for mx := 0; mx < mxx; mx++ {
			for i := 0; i < nComp; i++ {
				compIndex := scan[i].CompIndex
				hi := d.comp[compIndex].H
				vi := d.comp[compIndex].V
				for j := 0; j < hi*vi; j++ {
					gg++
					// The blocks are traversed one MCU at a time. For 4:2:0 chroma
					// subsampling, there are four Y 8x8 blocks in every 16x16 MCU.
					//
					// For a sequential 32x16 pixel image, the Y blocks visiting order is:
					//	0 1 4 5
					//	2 3 6 7
					//
					// For progressive images, the interleaved scans (those with nComp > 1)
					// are traversed as above, but non-interleaved scans are traversed left
					// to right, top to bottom:
					//	0 1 2 3
					//	4 5 6 7
					// Only DC scans (zigStart == 0) can be interleaved. AC scans must have
					// only one Component.
					//
					// To further complicate matters, for non-interleaved scans, there is no
					// data for any blocks that are inside the image at the MCU level but
					// outside the image at the pixel level. For example, a 24x16 pixel 4:2:0
					// progressive image consists of two 16x16 MCUs. The interleaved scans
					// will process 8 Y blocks:
					//	0 1 4 5
					//	2 3 6 7
					// The non-interleaved scans will process only 6 Y blocks:
					//	0 1 2
					//	3 4 5
					if nComp != 1 {
						bx = hi*mx + j%hi
						by = vi*my + j/hi
					} else {
						q := mxx * hi
						bx = blockCount % q
						by = blockCount / q
						blockCount++
						if bx*8 >= d.width || by*8 >= d.height {
							continue
						}
					}

					var bitstream []BitstreamItem

					// Load the previous partially decoded coefficients, if applicable.
					if d.progressive {
						b = d.progCoeffs[compIndex][by*mxx*hi+bx]
					} else {
						b = Block{}
					}

					if ah != 0 {
						if err := d.refine(&b, &d.huff[AcTable][scan[i].Ta], zigStart, zigEnd, 1<<al); err != nil {
							return err
						}
					} else {
						zig := zigStart
						if zig == 0 {
							zig++
							// Decode the DC coefficient, as specified in section F.2.2.1.
							value, code, bitsLen, err := d.decodeHuffman(&d.huff[DcTable][scan[i].Td])
							if err != nil {
								return err
							}
							if value > 16 {
								return UnsupportedError("excessive DC Component")
							}

							dcDelta, extend, err := d.receiveExtend(value)
							if err != nil {
								return err
							}
							item := BitstreamItem{
								Code:        code,
								CodeBitsLen: bitsLen,
								Extend:      extend,
							}
							bitstream = append(bitstream, item)
							dc[compIndex] += dcDelta
							b[0] = dc[compIndex] << al
						}

						if zig <= zigEnd && d.eobRun > 0 { // cond0
							d.eobRun--
						} else { // cond1
							// Decode the AC coefficients, as specified in section F.2.2.2.
							huff := &d.huff[AcTable][scan[i].Ta]
							for ; zig <= zigEnd; zig++ {
								value, code, bitsLen, err := d.decodeHuffman(huff)
								if err != nil {
									return err
								}

								val0 := value >> 4
								val1 := value & 0x0f
								if val1 != 0 { // cond2
									zig += int32(val0)
									if zig > zigEnd { // cond3
										break
									}
									// cond4
									ac, extend, err := d.receiveExtend(val1)
									if err != nil {
										return err
									}
									b[Unzig[zig]] = ac << al

									//if gg == GG {
									//	if zig == 33 {
									//		fmt.Println("k0:", zig, val0, uint32(ac))
									//	}
									//	fmt.Printf("xx: %v %v %v\n", zig, val0, uint32(ac))
									//}

									item := BitstreamItem{
										Code:          code,
										CodeBitsLen:   bitsLen,
										Extend:        extend,
										ExtendBitsLen: val1,
									}
									bitstream = append(bitstream, item)

								} else { // cond5
									item := BitstreamItem{
										Code:        code,
										CodeBitsLen: bitsLen,
										Extend:      0,
									}
									bitstream = append(bitstream, item)

									if val0 != 0x0f { // cond6
										d.eobRun = uint16(1 << val0)
										if val0 != 0 { // cond7
											bits, err := d.decodeBits(int32(val0))
											if err != nil {
												return err
											}
											d.eobRun |= uint16(bits)

											item := BitstreamItem{
												Code:        uint16(bits),
												CodeBitsLen: bitsLen,
												Extend:      0,
											}
											bitstream = append(bitstream, item)
										}
										d.eobRun--
										break
									}
									// cond8
									zig += 0x0f
								}
							}
						}
					}

					if d.progressive {
						// Save the coefficients.
						d.progCoeffs[compIndex][by*mxx*hi+bx] = b
						// At this point, we could call reconstructBlock to dequantize and perform the
						// inverse DCT, to save early stages of a progressive image to the *image.YCbCr
						// buffers (the whole point of progressive encoding), but in Go, the jpeg.Decode
						// function does not return until the entire image is decoded, so we "continue"
						// here to avoid waxsted computation. Instead, reconstructBlock is called on each
						// accumulated Block by the reconstructProgressiveImage method after all of the
						// SOS markers are processed.
						continue
					}
					if d.aux.BitstreamItems[compIndex] == nil {
						d.aux.BitstreamItems[compIndex] = make([][]BitstreamItem, 0)
					}

					d.aux.BitstreamItems[compIndex] = append(d.aux.BitstreamItems[compIndex], bitstream)
					d.aux.ComponentBlocks[compIndex] = append(d.aux.ComponentBlocks[compIndex], b)

					//if gg == GG {
					//	//fmt.Printf("b0: %v\n", b)
					//	fmt.Printf("b: ")
					//	for kk := 0; kk < 64; kk++ {
					//		fmt.Printf("%d ", uint32(b[kk]))
					//	}
					//	fmt.Println("")
					//}

					if err := d.reconstructBlock(&b, bx, by, int(compIndex)); err != nil {
						return err
					}
				} // for j
			} // for i
			mcu++
			if d.ri > 0 && mcu%d.ri == 0 && mcu < mxx*myy {
				// For well-formed input, the RST[0-7] restart marker follows
				// immediately. For corrupt input, call findRST to try to
				// resynchronize.
				if err := d.readFull(d.tmp[:2]); err != nil {
					return err
				} else if d.tmp[0] != 0xff || d.tmp[1] != expectedRST {
					if err := d.findRST(expectedRST); err != nil {
						return err
					}
				}
				expectedRST++
				if expectedRST == rst7Marker+1 {
					expectedRST = rst0Marker
				}
				// Reset the Huffman decoder.
				d.bits = bits{}
				// Reset the DC components, as per section F.2.1.3.1.
				dc = [maxComponents]int32{}
				// Reset the progressive decoder state, as per section G.1.2.2.
				d.eobRun = 0
			}
		} // for mx
	} // for my

	return nil
}

// refine decodes a successive approximation refinement Block, as specified in
// section G.1.2.
func (d *decoder) refine(b *Block, h *Huffman, zigStart, zigEnd, delta int32) error {
	// Refining a DC Component is trivial.
	if zigStart == 0 {
		if zigEnd != 0 {
			panic("unreachable")
		}
		bit, err := d.decodeBit()
		if err != nil {
			return err
		}
		if bit {
			b[0] |= delta
		}
		return nil
	}

	// Refining AC components is more complicated; see sections G.1.2.2 and G.1.2.3.
	zig := zigStart
	if d.eobRun == 0 {
	loop:
		for ; zig <= zigEnd; zig++ {
			z := int32(0)
			value, _, _, err := d.decodeHuffman(h)
			if err != nil {
				return err
			}
			val0 := value >> 4
			val1 := value & 0x0f

			switch val1 {
			case 0:
				if val0 != 0x0f {
					d.eobRun = uint16(1 << val0)
					if val0 != 0 {
						bits, err := d.decodeBits(int32(val0))
						if err != nil {
							return err
						}
						d.eobRun |= uint16(bits)
					}
					break loop
				}
			case 1:
				z = delta
				bit, err := d.decodeBit()
				if err != nil {
					return err
				}
				if !bit {
					z = -z
				}
			default:
				return FormatError("unexpected Huffman code")
			}

			zig, err = d.refineNonZeroes(b, zig, zigEnd, int32(val0), delta)
			if err != nil {
				return err
			}
			if zig > zigEnd {
				return FormatError("too many coefficients")
			}
			if z != 0 {
				b[Unzig[zig]] = z
			}
		}
	}
	if d.eobRun > 0 {
		d.eobRun--
		if _, err := d.refineNonZeroes(b, zig, zigEnd, -1, delta); err != nil {
			return err
		}
	}
	return nil
}

// refineNonZeroes refines non-zero entries of b in zig-zag order. If nz >= 0,
// the first nz zero entries are skipped over.
func (d *decoder) refineNonZeroes(b *Block, zig, zigEnd, nz, delta int32) (int32, error) {
	for ; zig <= zigEnd; zig++ {
		u := Unzig[zig]
		if b[u] == 0 {
			if nz == 0 {
				break
			}
			nz--
			continue
		}
		bit, err := d.decodeBit()
		if err != nil {
			return 0, err
		}
		if !bit {
			continue
		}
		if b[u] >= 0 {
			b[u] += delta
		} else {
			b[u] -= delta
		}
	}
	return zig, nil
}

func (d *decoder) reconstructProgressiveImage() error {
	// The h0, mxx, by and bx variables have the same meaning as in the
	// processSOS method.
	h0 := d.comp[0].H
	mxx := (d.width + 8*h0 - 1) / (8 * h0)
	for i := 0; i < d.nComp; i++ {
		if d.progCoeffs[i] == nil {
			continue
		}
		v := 8 * d.comp[0].V / d.comp[i].V
		h := 8 * d.comp[0].H / d.comp[i].H
		stride := mxx * d.comp[i].H
		for by := 0; by*v < d.height; by++ {
			for bx := 0; bx*h < d.width; bx++ {
				if err := d.reconstructBlock(&d.progCoeffs[i][by*stride+bx], bx, by, i); err != nil {
					return err
				}
			}
		}
	}
	return nil
}

// reconstructBlock dequantizes, performs the inverse DCT and stores the Block
// to the image.
func (d *decoder) reconstructBlock(b *Block, bx, by, compIndex int) error {
	var tmp Block
	copy(tmp[:], b[:])
	d.aux.ComponentBlocks[compIndex] = append(d.aux.ComponentBlocks[compIndex], tmp)

	qt := &d.quant[d.comp[compIndex].Tq]
	for zig := 0; zig < BlockSize; zig++ {
		b[Unzig[zig]] *= qt[zig]
	}

	//if gg == 1 {
	//	fmt.Printf("qt: ")
	//	for kk := 0; kk < 64; kk++ {
	//		fmt.Printf("%d ", uint32(qt[kk]))
	//	}
	//	fmt.Println("")
	//}

	idct(b)

	//if gg == GG {
	//	//fmt.Printf("b0: %v\n", b)
	//	fmt.Printf("res: ")
	//	for kk := 0; kk < 64; kk++ {
	//		fmt.Printf("%d ", uint32(b[kk]))
	//	}
	//	fmt.Println("")
	//}

	dst, stride := []byte(nil), 0
	if d.nComp == 1 {
		dst, stride = d.img1.Pix[8*(by*d.img1.Stride+bx):], d.img1.Stride
	} else {
		switch compIndex {
		case 0:
			dst, stride = d.img3.Y[8*(by*d.img3.YStride+bx):], d.img3.YStride
		case 1:
			dst, stride = d.img3.Cb[8*(by*d.img3.CStride+bx):], d.img3.CStride
		case 2:
			dst, stride = d.img3.Cr[8*(by*d.img3.CStride+bx):], d.img3.CStride
		case 3:
			dst, stride = d.blackPix[8*(by*d.blackStride+bx):], d.blackStride
		default:
			return UnsupportedError("too many components")
		}
	}
	// Level shift by +128, clip to [0, 255], and write to dst.
	var tmp2 [8 * 8]int32
	for y := 0; y < 8; y++ {
		y8 := y * 8
		yStride := y * stride
		for x := 0; x < 8; x++ {
			c := b[y8+x]
			if c < -128 {
				c = 0
			} else if c > 127 {
				c = 255
			} else {
				c += 128
			}
			dst[yStride+x] = uint8(c)
			tmp2[y*8+x] = c
		}
	}

	//if gg == GG {
	//	fmt.Printf("pos: %v\n", 8*(by*d.img3.YStride+bx))
	//
	//	fmt.Printf("res: ")
	//	for kk := 0; kk < 64; kk++ {
	//		fmt.Printf("%d ", uint32(tmp2[kk]))
	//	}
	//	fmt.Println("")
	//}

	return nil
}

// findRST advances past the next RST restart marker that matches expectedRST.
// Other than I/O errors, it is also an error if we encounter an {0xFF, M}
// two-byte marker sequence where M is not 0x00, 0xFF or the expectedRST.
//
// This is similar to libjpeg's jdmarker.C's next_marker function.
// https://github.com/libjpeg-turbo/libjpeg-turbo/blob/2dfe6c0fe9e18671105e94f7cbf044d4a1d157e6/jdmarker.c#L892-L935
//
// Precondition: d.tmp[:2] holds the next two bytes of JPEG-encoded input
// (input in the d.readFull sense).
func (d *decoder) findRST(expectedRST uint8) error {
	for {
		// i is the index such that, at the bottom of the loop, we read 2-i
		// bytes into d.tmp[i:2], maintaining the invariant that d.tmp[:2]
		// holds the next two bytes of JPEG-encoded input. It is either 0 or 1,
		// so that each iteration advances by 1 or 2 bytes (or returns).
		i := 0

		if d.tmp[0] == 0xff {
			if d.tmp[1] == expectedRST {
				return nil
			} else if d.tmp[1] == 0xff {
				i = 1
			} else if d.tmp[1] != 0x00 {
				// libjpeg's jdmarker.C's jpeg_resync_to_restart does something
				// fancy here, treating RST markers within two (modulo 8) of
				// expectedRST differently from RST markers that are 'more
				// distant'. Until we see evidence that recovering from such
				// cases is frequent enough to be worth the complexity, we take
				// a simpler approach for now. Any marker that's not 0x00, 0xff
				// or expectedRST is a fatal FormatError.
				return FormatError("bad RST marker")
			}

		} else if d.tmp[1] == 0xff {
			d.tmp[0] = 0xff
			i = 1
		}

		if err := d.readFull(d.tmp[i:2]); err != nil {
			return err
		}
	}
}

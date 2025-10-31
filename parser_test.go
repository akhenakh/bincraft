package bincraft_test

import (
	"encoding/binary"
	"os"
	"path/filepath"
	"testing"
	"time"

	"github.com/akhenakh/bincraft" // Update with your module path

	"github.com/klauspost/compress/zstd"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// loadAndDecompressTestData is a helper function to read and decompress the golden file.
func loadAndDecompressTestData(t *testing.T) []byte {
	// Re-generate this file if the parser logic changes significantly
	// curl -s 'https://adsb.lol/re-api/?binCraft&zstd&box=40,50,-80,-70' -o pkg/bincraft/testdata/sample.zst
	path := filepath.Join("testdata", "sample.zst")
	compressedData, err := os.ReadFile(path)
	require.NoError(t, err, "should be able to read the test data file")
	require.NotEmpty(t, compressedData, "test data file should not be empty")

	reader, err := zstd.NewReader(nil)
	require.NoError(t, err, "should be able to create zstd reader")
	defer reader.Close()

	decompressedData, err := reader.DecodeAll(compressedData, nil)
	require.NoError(t, err, "should be able to decompress test data")
	require.NotEmpty(t, decompressedData, "decompressed data should not be empty")

	return decompressedData
}

func TestParse_GoldenFile(t *testing.T) {
	decompressedData := loadAndDecompressTestData(t)
	globeData, err := bincraft.Parse(decompressedData)

	assert.NoError(t, err, "parsing the golden file should not produce an error")
	require.NotNil(t, globeData, "parsed globeData should not be nil")

	// Test Header Fields
	assert.True(t, globeData.Now > float64(time.Date(2022, 1, 1, 0, 0, 0, 0, time.UTC).Unix()))
	assert.Positive(t, globeData.AircraftWithPosition, "header should report a positive number of aircraft globally")
	assert.NotZero(t, globeData.Bounds.North, "bounds should be parsed from header")
	assert.NotZero(t, globeData.Bounds.South, "bounds should be parsed from header")

	// Test Aircraft Data
	require.NotEmpty(t, globeData.Aircraft, "aircraft list should not be empty after parsing")

	// The header reports a GLOBAL aircraft count. The body contains a LOCAL count for the requested box.
	parsedCount := uint32(len(globeData.Aircraft))
	globalCount := globeData.AircraftWithPosition
	assert.LessOrEqual(t, parsedCount, globalCount, "parsed aircraft count (local) should be less than or equal to the header count (global)")

	// Spot-check the first aircraft for plausible values
	firstAircraft := globeData.Aircraft[0]
	assert.Len(t, firstAircraft.Hex, 6)
	assert.Regexp(t, `^[a-f0-9]{6}$`, firstAircraft.Hex)
	assert.True(t, firstAircraft.Lat >= -90.0 && firstAircraft.Lat <= 90.0, "latitude should be valid")
	assert.True(t, firstAircraft.Lon >= -180.0 && firstAircraft.Lon <= 180.0, "longitude should be valid")

	_, isInt32 := firstAircraft.AltBaro.(int32)
	_, isString := firstAircraft.AltBaro.(string)
	assert.True(t, isInt32 || (isString && firstAircraft.AltBaro == "ground"), "alt_baro should be an int32 or 'ground'")
}

func TestMarshalAndParse_RoundTrip(t *testing.T) {
	decompressedData := loadAndDecompressTestData(t)

	// 1. Parse original data from the golden file
	originalGlobeData, err := bincraft.Parse(decompressedData)
	require.NoError(t, err)
	require.NotNil(t, originalGlobeData)
	require.NotEmpty(t, originalGlobeData.Aircraft)

	// 2. Marshal it back to a binary format using our new function
	marshaledData, err := originalGlobeData.MarshalBinary()
	require.NoError(t, err)
	require.NotEmpty(t, marshaledData)
	require.Equal(t, bincraft.DefaultStride+len(originalGlobeData.Aircraft)*bincraft.DefaultStride, len(marshaledData))

	// 3. Parse the newly generated binary data
	var roundTripGlobeData bincraft.GlobeData
	err = roundTripGlobeData.UnmarshalBinary(marshaledData)
	require.NoError(t, err)
	require.NotNil(t, roundTripGlobeData)

	// 4. Compare headers
	assert.InDelta(t, originalGlobeData.Now, roundTripGlobeData.Now, 0.001, "Timestamp should be nearly identical")
	assert.Equal(t, uint32(bincraft.DefaultStride), roundTripGlobeData.Stride, "Stride should be the default value")
	assert.Equal(t, originalGlobeData.AircraftWithPosition, roundTripGlobeData.AircraftWithPosition)
	assert.Equal(t, originalGlobeData.GlobeIndex, roundTripGlobeData.GlobeIndex)
	assert.Equal(t, originalGlobeData.Bounds, roundTripGlobeData.Bounds)

	// 5. Compare aircraft lists
	require.Equal(t, len(originalGlobeData.Aircraft), len(roundTripGlobeData.Aircraft))

	// Spot-check the first aircraft to ensure field integrity after a round trip
	// A full deep comparison is complex due to precision loss in the format.
	originalAc := originalGlobeData.Aircraft[0]
	roundTripAc := roundTripGlobeData.Aircraft[0]

	assert.Equal(t, originalAc.Hex, roundTripAc.Hex)
	assert.InDelta(t, originalAc.Lat, roundTripAc.Lat, 1e-6)
	assert.InDelta(t, originalAc.Lon, roundTripAc.Lon, 1e-6)
	assert.Equal(t, originalAc.Type, roundTripAc.Type)
	assert.Equal(t, originalAc.Flight, roundTripAc.Flight)
	assert.Equal(t, originalAc.Squawk, roundTripAc.Squawk)

	// Check altitude, accounting for "ground" case and scaling precision loss
	if originalAc.AltBaro == "ground" {
		assert.Equal(t, "ground", roundTripAc.AltBaro)
	} else {
		originalAlt, ok1 := originalAc.AltBaro.(int32)
		roundTripAlt, ok2 := roundTripAc.AltBaro.(int32)
		require.True(t, ok1 && ok2, "AltBaro should be int32 for airborne aircraft")
		assert.InDelta(t, originalAlt, roundTripAlt, 25.0, "Barometric altitude has a precision of 25ft")
	}

	// Check a rate value, accounting for scaling precision loss
	assert.InDelta(t, originalAc.BaroRate, roundTripAc.BaroRate, 8.0, "Baro rate has a precision of 8ft/min")
}

func TestParse_EdgeCases(t *testing.T) {
	t.Run("NilData", func(t *testing.T) {
		_, err := bincraft.Parse(nil)
		assert.Error(t, err, "parsing nil data should return an error")
	})

	t.Run("TooShortData", func(t *testing.T) {
		shortData := []byte{0x01, 0x02, 0x03, 0x04}
		_, err := bincraft.Parse(shortData)
		assert.Error(t, err, "parsing data shorter than header should return an error")
	})

	t.Run("HeaderOnly", func(t *testing.T) {
		header := make([]byte, 112)
		binary.LittleEndian.PutUint32(header[8:12], 112) // Stride

		globeData, err := bincraft.Parse(header)
		assert.NoError(t, err, "parsing header-only data should not error")
		require.NotNil(t, globeData)
		assert.Empty(t, globeData.Aircraft, "aircraft list should be empty for header-only data")
	})
}

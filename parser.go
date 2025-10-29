// Package bincraft provides a parser for the custom binary format ("binCraft")
// used by adsb.lol for efficient real-time aircraft data transmission.
package bincraft

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"math"
)

// GlobeData represents the top-level structure of the parsed binary data.
// It contains metadata about the data batch and a slice of aircraft.
type GlobeData struct {
	// Now is the timestamp when the data was generated, in seconds since the Unix epoch.
	Now float64 `json:"now"`
	// AircraftWithPosition is the total number of aircraft with positions globally.
	AircraftWithPosition uint32 `json:"aircraft_with_position"`
	// GlobeIndex is an identifier for the data batch.
	GlobeIndex uint32 `json:"globe_index"`
	// Stride is the size in bytes of a single aircraft record in the binary payload.
	Stride uint32 `json:"-"`
	// Bounds represents the geographic bounding box for the aircraft data in this payload.
	Bounds Bounds `json:"bounds"`
	// Aircraft is the list of aircraft parsed from the payload.
	Aircraft []Aircraft `json:"aircraft"`
}

// Bounds represents the geographic bounding box.
type Bounds struct {
	South int16 `json:"south"`
	West  int16 `json:"west"`
	North int16 `json:"north"`
	East  int16 `json:"east"`
}

// Aircraft holds the decoded data for a single aircraft.
type Aircraft struct {
	// Hex is the ICAO 24-bit identifier of the aircraft, as a hex string.
	Hex string `json:"hex"`
	// SeenPos is the time in seconds since the last position update was received.
	SeenPos float64 `json:"seen_pos"`
	// Seen is the time in seconds since any message was last received from this aircraft.
	Seen float64 `json:"seen"`
	// Lat is the latitude of the aircraft in degrees.
	Lat float64 `json:"lat"`
	// Lon is the longitude of the aircraft in degrees.
	Lon float64 `json:"lon"`
	// AltBaro is the barometric altitude in feet, or the string "ground" if the aircraft is on the ground.
	AltBaro interface{} `json:"alt_baro"`
	// AltGeom is the geometric (WGS84) altitude in feet.
	AltGeom int32 `json:"alt_geom"`
	// BaroRate is the barometric vertical rate in feet per minute.
	BaroRate int16 `json:"baro_rate"`
	// GeomRate is the geometric vertical rate in feet per minute.
	GeomRate int16 `json:"geom_rate"`
	// NavAltitudeMCP is the autopilot selected altitude in feet from the Mode Control Panel (MCP) or Flight Control Unit (FCU).
	NavAltitudeMCP uint16 `json:"nav_altitude_mcp"`
	// NavAltitudeFMS is the Flight Management System (FMS) selected altitude in feet.
	NavAltitudeFMS uint16 `json:"nav_altitude_fms"`
	// NavQNH is the altimeter setting (QNH) in hPa (millibars).
	NavQNH float64 `json:"nav_qnh"`
	// NavHeading is the heading selected on the MCP/FCU, in degrees.
	NavHeading float64 `json:"nav_heading"`
	// Squawk is the transponder code, as a 4-digit octal string.
	Squawk string `json:"squawk"`
	// GroundSpeed is the ground speed in knots.
	GroundSpeed float64 `json:"gs"`
	// Mach is the Mach number.
	Mach float64 `json:"mach"`
	// Roll is the roll angle in degrees. Negative values indicate left roll.
	Roll float64 `json:"roll"`
	// Track is the ground track in degrees.
	Track float64 `json:"track"`
	// TrackRate is the rate of turn in degrees per second.
	TrackRate float64 `json:"track_rate"`
	// MagHeading is the magnetic heading in degrees.
	MagHeading float64 `json:"mag_heading"`
	// TrueHeading is the true heading in degrees.
	TrueHeading float64 `json:"true_heading"`
	// WindDirection is the calculated wind direction in degrees.
	WindDirection int16 `json:"wd"`
	// WindSpeed is the calculated wind speed in knots.
	WindSpeed int16 `json:"ws"`
	// OAT is the Outside Air Temperature in degrees Celsius.
	OAT int16 `json:"oat"`
	// TAT is the Total Air Temperature in degrees Celsius.
	TAT int16 `json:"tat"`
	// TAS is the True Airspeed in knots.
	TAS uint16 `json:"tas"`
	// IAS is the Indicated Airspeed in knots.
	IAS uint16 `json:"ias"`
	// RC is the reply capability.
	RC uint16 `json:"rc"`
	// Messages is the total number of messages received from this aircraft.
	Messages uint16 `json:"messages"`
	// Category is the aircraft category, represented as a hex string (e.g., "A5").
	Category string `json:"category,omitempty"`
	// NIC is the Navigation Integrity Category.
	NIC uint8 `json:"nic"`
	// Emergency indicates the emergency status (e.g., 0 for none).
	Emergency uint8 `json:"emergency"`
	// Type indicates the source of the message (e.g., "adsb_icao", "mlat").
	Type string `json:"type"`
	// Airground indicates the air/ground status of the aircraft (e.g., 0 for airborne, 1 for on ground).
	Airground uint8 `json:"airground"`
	// NavAltitudeSrc indicates the source of the navigation altitude data.
	NavAltitudeSrc uint8 `json:"nav_altitude_src"`
	// SILType is the Source Integrity Level type.
	SILType uint8 `json:"sil_type"`
	// ADSBVersion is the ADS-B version number (e.g., 0, 1, 2).
	ADSBVersion uint8 `json:"adsb_version"`
	// ADSRVersion is the ADS-R version number.
	ADSRVersion uint8 `json:"adsr_version"`
	// TISBVersion is the TIS-B version number.
	TISBVersion uint8 `json:"tisb_version"`
	// NACP is the Navigation Accuracy Category for Position.
	NACP uint8 `json:"nac_p"`
	// NACV is the Navigation Accuracy Category for Velocity.
	NACV uint8 `json:"nac_v"`
	// SIL is the Source Integrity Level.
	SIL uint8 `json:"sil"`
	// GVA is the Geometric Vertical Accuracy.
	GVA uint8 `json:"gva"`
	// SDA is the System Design Assurance.
	SDA uint8 `json:"sda"`
	// NICA is the Navigation Integrity Category Supplement A.
	NICA uint8 `json:"nic_a"`
	// NICC is the Navigation Integrity Category Supplement C.
	NICC uint8 `json:"nic_c"`
	// RSSI is the Received Signal Strength Indicator in dBFS.
	RSSI float64 `json:"rssi"`
	// DBFlags are flags from the database.
	DBFlags uint8 `json:"dbFlags"`
	// Flight is the callsign or flight number.
	Flight string `json:"flight"`
	// TypeCode is the ICAO aircraft type designator (e.g., "A320", "B738").
	TypeCode string `json:"t"`
	// Registration is the aircraft registration number (tail number).
	Registration string `json:"r"`
	// ReceiverCount is the number of receivers currently seeing this aircraft.
	ReceiverCount uint8 `json:"receiverCount"`
	// NICBaro is the Navigation Integrity Category for barometric altitude.
	NICBaro uint8 `json:"nic_baro"`
	// Alert is a flag indicating a transponder alert.
	Alert uint8 `json:"alert"`
	// SPI is a flag indicating Special Position Identification.
	SPI uint8 `json:"spi"`
	// NavModes is a list of active navigation modes (e.g., "autopilot", "vnav").
	NavModes []string `json:"nav_modes"`
}

// aircraftTypeMap maps a numeric type identifier to its string representation.
var aircraftTypeMap = map[uint8]string{
	0: "adsb_icao", 1: "adsb_icao_nt", 2: "adsr_icao", 3: "tisb_icao",
	4: "adsc", 5: "mlat", 6: "other", 7: "mode_s", 8: "adsb_other",
	9: "adsr_other", 10: "tisb_trackfile", 11: "tisb_other", 12: "mode_ac",
}

// cleanStr converts a byte slice to a string, trimming null terminators
// and filtering out non-printable characters.
func cleanStr(b []byte) string {
	// Find the first null byte, which indicates the end of a C-style string.
	if i := bytes.IndexByte(b, 0); i != -1 {
		b = b[:i]
	}
	var result []byte
	// Filter for printable ASCII characters.
	for _, char := range b {
		if char > 32 && char < 127 {
			result = append(result, char)
		}
	}
	return string(result)
}

// Parse decodes the binCraft binary data format into a GlobeData struct.
func Parse(data []byte) (*GlobeData, error) {
	// The header is at least 28 bytes long.
	if len(data) < 28 {
		return nil, fmt.Errorf("data too short for header")
	}
	// The stride is the length of each aircraft record, located at offset 8.
	stride := binary.LittleEndian.Uint32(data[8:12])
	if stride == 0 {
		return nil, fmt.Errorf("invalid stride")
	}
	// The timestamp is a 64-bit value split into two 32-bit integers.
	nowLow := binary.LittleEndian.Uint32(data[0:4])
	nowHigh := binary.LittleEndian.Uint32(data[4:8])
	acWithPosCount := binary.LittleEndian.Uint32(data[12:16])
	globeIndex := binary.LittleEndian.Uint32(data[16:20])
	result := &GlobeData{
		// Reconstruct the full 64-bit timestamp.
		Now:                  float64(nowLow)/1000.0 + float64(nowHigh)*4294967.296,
		AircraftWithPosition: acWithPosCount,
		GlobeIndex:           globeIndex,
		Stride:               stride,
		Bounds:               Bounds{ /* ... */ }, // Bounds are parsed in a different part of the codebase not shown here.
		// Pre-allocate slice capacity for efficiency.
		Aircraft: make([]Aircraft, 0, (len(data)-int(stride))/int(stride)),
	}

	// Loop through the data, processing one aircraft record (chunk) at a time.
	// The first aircraft record starts at an offset equal to the stride.
	for offset := stride; offset+stride <= uint32(len(data)); offset += stride {
		chunk := data[offset : offset+stride]
		ac := Aircraft{}

		// --- Decode Fixed-Width Fields ---

		// The ICAO address is the lower 24 bits of a 32-bit integer.
		addr := binary.LittleEndian.Uint32(chunk[0:4])
		ac.Hex = fmt.Sprintf("%06x", addr&0xFFFFFF)

		ac.SeenPos = float64(binary.LittleEndian.Uint16(chunk[4:6])) / 10.0
		ac.Seen = float64(binary.LittleEndian.Uint16(chunk[6:8])) / 10.0
		// Latitude and longitude are stored as scaled integers.
		ac.Lon = float64(int32(binary.LittleEndian.Uint32(chunk[8:12]))) / 1e6
		ac.Lat = float64(int32(binary.LittleEndian.Uint32(chunk[12:16]))) / 1e6

		// Rates and altitudes are also scaled.
		ac.BaroRate = int16(binary.LittleEndian.Uint16(chunk[16:18])) * 8
		ac.GeomRate = int16(binary.LittleEndian.Uint16(chunk[18:20])) * 8
		altBaroVal := int32(int16(binary.LittleEndian.Uint16(chunk[20:22]))) * 25
		ac.AltGeom = int32(int16(binary.LittleEndian.Uint16(chunk[22:24]))) * 25

		ac.NavAltitudeMCP = binary.LittleEndian.Uint16(chunk[24:26]) * 4
		ac.NavAltitudeFMS = binary.LittleEndian.Uint16(chunk[26:28]) * 4
		ac.NavQNH = float64(int16(binary.LittleEndian.Uint16(chunk[28:30]))) / 10.0
		ac.NavHeading = float64(int16(binary.LittleEndian.Uint16(chunk[30:32]))) / 90.0
		ac.Squawk = fmt.Sprintf("%04x", binary.LittleEndian.Uint16(chunk[32:34]))
		ac.GroundSpeed = float64(int16(binary.LittleEndian.Uint16(chunk[34:36]))) / 10.0
		ac.Mach = float64(int16(binary.LittleEndian.Uint16(chunk[36:38]))) / 1000.0
		ac.Roll = float64(int16(binary.LittleEndian.Uint16(chunk[38:40]))) / 100.0
		ac.Track = float64(int16(binary.LittleEndian.Uint16(chunk[40:42]))) / 90.0
		ac.TrackRate = float64(int16(binary.LittleEndian.Uint16(chunk[42:44]))) / 100.0
		ac.MagHeading = float64(int16(binary.LittleEndian.Uint16(chunk[44:46]))) / 90.0
		ac.TrueHeading = float64(int16(binary.LittleEndian.Uint16(chunk[46:48]))) / 90.0
		ac.WindDirection = int16(binary.LittleEndian.Uint16(chunk[48:50]))
		ac.WindSpeed = int16(binary.LittleEndian.Uint16(chunk[50:52]))
		ac.OAT = int16(binary.LittleEndian.Uint16(chunk[52:54]))
		ac.TAT = int16(binary.LittleEndian.Uint16(chunk[54:56]))
		ac.TAS = binary.LittleEndian.Uint16(chunk[56:58])
		ac.IAS = binary.LittleEndian.Uint16(chunk[58:60])
		ac.RC = binary.LittleEndian.Uint16(chunk[60:62])
		ac.Messages = binary.LittleEndian.Uint16(chunk[62:64])

		// --- Decode Bit-Packed Fields ---

		// byte68 contains Airground status (lower 4 bits) and NavAltitudeSrc (upper 4 bits).
		byte68 := chunk[68]
		ac.Airground = byte68 & 0x0F // Mask for the lower 4 bits
		if ac.Airground == 1 {
			ac.AltBaro = "ground"
		} else {
			ac.AltBaro = altBaroVal
		}
		ac.NavAltitudeSrc = (byte68 & 0xF0) >> 4 // Mask for the upper 4 bits and shift

		rawCategory := chunk[64]
		if rawCategory != 0 {
			ac.Category = fmt.Sprintf("%X", rawCategory)
		}
		ac.NIC = chunk[65]

		// byte67 contains Emergency (lower 4 bits) and Type (upper 4 bits).
		byte67 := chunk[67]
		ac.Emergency = byte67 & 0x0F
		ac.Type = aircraftTypeMap[(byte67&0xF0)>>4]

		// byte69 contains SILType (lower 4 bits) and ADSBVersion (upper 4 bits).
		byte69 := chunk[69]
		ac.SILType = byte69 & 0x0F
		ac.ADSBVersion = (byte69 & 0xF0) >> 4

		// byte70 contains ADSRVersion (lower 4 bits) and TISBVersion (upper 4 bits).
		byte70 := chunk[70]
		ac.ADSRVersion = byte70 & 0x0F
		ac.TISBVersion = (byte70 & 0xF0) >> 4

		// byte71 contains NACP (lower 4 bits) and NACV (upper 4 bits).
		byte71 := chunk[71]
		ac.NACP = byte71 & 0x0F
		ac.NACV = (byte71 & 0xF0) >> 4

		// byte72 contains multiple 2-bit fields.
		byte72 := chunk[72]
		ac.SIL = byte72 & 0x03         // bits 0-1
		ac.GVA = (byte72 & 0x0C) >> 2  // bits 2-3
		ac.SDA = (byte72 & 0x30) >> 4  // bits 4-5
		ac.NICA = (byte72 & 0x40) >> 6 // bit 6
		ac.NICC = (byte72 & 0x80) >> 7 // bit 7

		// byte73 contains multiple 1-bit flags.
		byte73 := chunk[73]
		ac.NICBaro = byte73 & 1      // bit 0
		ac.Alert = (byte73 & 2) >> 1 // bit 1
		ac.SPI = (byte73 & 4) >> 2   // bit 2

		// The RSSI formula is specific to this data source.
		rawRSSI := float64(chunk[86])
		ac.RSSI = 10.0 * math.Log10((rawRSSI*rawRSSI)/65025.0+1.125e-5)

		// --- Decode String and Other Fields ---

		ac.DBFlags = chunk[87]
		ac.Flight = cleanStr(chunk[78:87])
		ac.TypeCode = cleanStr(chunk[88:92])
		ac.Registration = cleanStr(chunk[92:104])
		ac.ReceiverCount = chunk[104]

		// byte66 is a bitmask for navigation modes.
		navModesByte := chunk[66]
		ac.NavModes = make([]string, 0)
		if (navModesByte & 0x01) != 0 {
			ac.NavModes = append(ac.NavModes, "autopilot")
		}
		if (navModesByte & 0x02) != 0 {
			ac.NavModes = append(ac.NavModes, "vnav")
		}
		if (navModesByte & 0x04) != 0 {
			ac.NavModes = append(ac.NavModes, "alt_hold")
		}
		if (navModesByte & 0x08) != 0 {
			ac.NavModes = append(ac.NavModes, "approach")
		}
		if (navModesByte & 0x10) != 0 {
			ac.NavModes = append(ac.NavModes, "lnav")
		}
		if (navModesByte & 0x20) != 0 {
			ac.NavModes = append(ac.NavModes, "tcas")
		}

		result.Aircraft = append(result.Aircraft, ac)
	}
	return result, nil
}

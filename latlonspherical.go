// Copyright 2019 Wilhelm Stoll. All rights reserved.
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file.

// Pure go implementation of geodetic (lat-/longitude) functions,
// transcribed in part from JavaScript originals by I{(C) Chris Veness 2011-2016}
// and published under the same MIT Licence**, see
// U{Latitude/Longitude<http://www.Movable-Type.co.UK/scripts/latlong.html>}.

package latlonspherical

import (
	"math"

	"github.com/wilhelmstoll/mathx"
)

// LatLon represents a point on the earth's surface at the specified latitude / longitude.
type LatLon struct {
	Lat float64
	Lon float64
}

// DistanceTo returns the distance from ‘this’ point to destination point (using haversine formula).
func (l LatLon) DistanceTo(point LatLon, radius float64) float64 {
	if radius == 0 {
		radius = 6371e3
	}

	// a = sin²(Δφ/2) + cos(φ1)⋅cos(φ2)⋅sin²(Δλ/2)
	// tanδ = √(a) / √(1−a)
	// see mathforum.org/library/drmath/view/51879.html for derivation

	R := radius
	φ1 := mathx.Rad(l.Lat)
	λ1 := mathx.Rad(l.Lon)
	φ2 := mathx.Rad(point.Lat)
	λ2 := mathx.Rad(point.Lon)
	Δφ := φ2 - φ1
	Δλ := λ2 - λ1

	a := math.Sin(Δφ/2)*math.Sin(Δφ/2) + math.Cos(φ1)*math.Cos(φ2)*math.Sin(Δλ/2)*math.Sin(Δλ/2)
	c := 2 * math.Atan2(math.Sqrt(a), math.Sqrt(1-a))
	d := R * c

	return d
}

// BearingTo returns the (initial) bearing from ‘this’ point to destination point.
func (l LatLon) BearingTo(point LatLon) float64 {
	// tanθ = sinΔλ⋅cosφ2 / cosφ1⋅sinφ2 − sinφ1⋅cosφ2⋅cosΔλ
	// see mathforum.org/library/drmath/view/55417.html for derivation

	φ1 := mathx.Rad(l.Lat)
	φ2 := mathx.Rad(point.Lat)
	Δλ := mathx.Rad(point.Lon - l.Lon)
	y := math.Sin(Δλ) * math.Cos(φ2)
	x := math.Cos(φ1)*math.Sin(φ2) - math.Sin(φ1)*math.Cos(φ2)*math.Cos(Δλ)
	θ := math.Atan2(y, x)

	return math.Mod((mathx.Deg(θ) + 360), 360)
}

// FinalBearingTo returns final bearing arriving at destination destination point from ‘this’ point; the final bearing
// will differ from the initial bearing by varying degrees according to distance and latitude.
func (l LatLon) FinalBearingTo(point LatLon) float64 {
	// get initial bearing from destination point to this point & reverse it by adding 180°
	return math.Mod((point.BearingTo(l) + 180), 360)
}

// MidpointTo returns the midpoint between ‘this’ point and the supplied point.
func (l LatLon) MidpointTo(point LatLon) LatLon {
	// φm = atan2( sinφ1 + sinφ2, √( (cosφ1 + cosφ2⋅cosΔλ) ⋅ (cosφ1 + cosφ2⋅cosΔλ) ) + cos²φ2⋅sin²Δλ )
	// λm = λ1 + atan2(cosφ2⋅sinΔλ, cosφ1 + cosφ2⋅cosΔλ)
	// see mathforum.org/library/drmath/view/51822.html for derivation

	φ1 := mathx.Rad(l.Lat)
	λ1 := mathx.Rad(l.Lon)
	φ2 := mathx.Rad(point.Lat)
	Δλ := mathx.Rad(point.Lon - l.Lon)

	Bx := math.Cos(φ2) * math.Cos(Δλ)
	By := math.Cos(φ2) * math.Sin(Δλ)

	x := math.Sqrt((math.Cos(φ1)+Bx)*(math.Cos(φ1)+Bx) + By*By)
	y := math.Sin(φ1) + math.Sin(φ2)
	φ3 := math.Atan2(y, x)

	λ3 := λ1 + math.Atan2(By, math.Cos(φ1)+Bx)

	return LatLon{mathx.Deg(φ3), math.Mod((mathx.Deg(λ3)+540), 360) - 180} // normalise to −180..+180°
}

// IntermediatePointTo returns the point at given fraction between ‘this’ point and specified point.
func (l LatLon) IntermediatePointTo(point LatLon, fraction float64) LatLon {
	φ1 := mathx.Rad(l.Lat)
	λ1 := mathx.Rad(l.Lon)
	φ2 := mathx.Rad(point.Lat)
	λ2 := mathx.Rad(point.Lon)
	sinφ1 := math.Sin(φ1)
	cosφ1 := math.Cos(φ1)
	sinλ1 := math.Sin(λ1)
	cosλ1 := math.Cos(λ1)
	sinφ2 := math.Sin(φ2)
	cosφ2 := math.Cos(φ2)
	sinλ2 := math.Sin(λ2)
	cosλ2 := math.Cos(λ2)

	// distance between points
	Δφ := φ2 - φ1
	Δλ := λ2 - λ1
	a := math.Sin(Δφ/2)*math.Sin(Δφ/2) + math.Cos(φ1)*math.Cos(φ2)*math.Sin(Δλ/2)*math.Sin(Δλ/2)
	δ := 2 * math.Atan2(math.Sqrt(a), math.Sqrt(1-a))

	A := math.Sin((1-fraction)*δ) / math.Sin(δ)
	B := math.Sin(fraction*δ) / math.Sin(δ)

	x := A*cosφ1*cosλ1 + B*cosφ2*cosλ2
	y := A*cosφ1*sinλ1 + B*cosφ2*sinλ2
	z := A*sinφ1 + B*sinφ2

	φ3 := math.Atan2(z, math.Sqrt(x*x+y*y))
	λ3 := math.Atan2(y, x)

	return LatLon{mathx.Deg(φ3), math.Mod((mathx.Deg(λ3)+540), 360) - 180} // normalise lon to −180..+180°
}

// DestinationPoint returns the destination point from ‘this’ point having travelled the given distance on the
// given initial bearing (bearing normally varies around path followed).
func (l LatLon) DestinationPoint(distance float64, bearing float64, radius float64) LatLon {
	if radius == 0 {
		radius = 6371e3
	}

	// sinφ2 = sinφ1⋅cosδ + cosφ1⋅sinδ⋅cosθ
	// tanΔλ = sinθ⋅sinδ⋅cosφ1 / cosδ−sinφ1⋅sinφ2
	// see mathforum.org/library/drmath/view/52049.html for derivation

	δ := distance / radius // angular distance in radians
	θ := mathx.Rad(bearing)

	φ1 := mathx.Rad(l.Lat)
	λ1 := mathx.Rad(l.Lon)

	sinφ1 := math.Sin(φ1)
	cosφ1 := math.Cos(φ1)
	sinδ := math.Sin(δ)
	cosδ := math.Cos(δ)
	sinθ := math.Sin(θ)
	cosθ := math.Cos(θ)

	sinφ2 := sinφ1*cosδ + cosφ1*sinδ*cosθ
	φ2 := math.Asin(sinφ2)
	y := sinθ * sinδ * cosφ1
	x := cosδ - sinφ1*sinφ2
	λ2 := λ1 + math.Atan2(y, x)

	return LatLon{mathx.Deg(φ2), math.Mod((mathx.Deg(λ2)+540), 360) - 180} // normalise to −180..+180°
}

// Intersection returns the point of intersection of two paths defined by point and bearing.
func (l LatLon) Intersection(p1 LatLon, brng1 float64, p2 LatLon, brng2 float64) LatLon {
	// see www.edwilliams.org/avform.htm#Intersection

	φ1 := mathx.Rad(p1.Lat)
	λ1 := mathx.Rad(p1.Lon)
	φ2 := mathx.Rad(p2.Lat)
	λ2 := mathx.Rad(p2.Lon)
	θ13 := mathx.Rad(brng1)
	θ23 := mathx.Rad(brng2)
	Δφ := φ2 - φ1
	Δλ := λ2 - λ1

	// angular distance p1-p2
	δ12 := 2 * math.Asin(math.Sqrt(math.Sin(Δφ/2)*math.Sin(Δφ/2)+math.Cos(φ1)*math.Cos(φ2)*math.Sin(Δλ/2)*math.Sin(Δλ/2)))
	if δ12 == 0 {
		return LatLon{}
	}

	// initial/final bearings between points
	cosθa := (math.Sin(φ2) - math.Sin(φ1)*math.Cos(δ12)) / (math.Sin(δ12) * math.Cos(φ1))
	cosθb := (math.Sin(φ1) - math.Sin(φ2)*math.Cos(δ12)) / (math.Sin(δ12) * math.Cos(φ2))
	θa := math.Acos(math.Min(math.Max(cosθa, -1), 1)) // protect against rounding errors
	θb := math.Acos(math.Min(math.Max(cosθb, -1), 1)) // protect against rounding errors

	var θ12 float64
	if math.Sin(λ2-λ1) > 0 {
		θ12 = θa
	} else {
		θ12 = 2*math.Pi - θa
	}

	var θ21 float64
	if math.Sin(λ2-λ1) > 0 {
		θ21 = 2*math.Pi - θb
	} else {
		θ21 = θb
	}

	α1 := θ13 - θ12 // angle 2-1-3
	α2 := θ21 - θ23 // angle 1-2-3

	if math.Sin(α1) == 0 && math.Sin(α2) == 0 {
		return LatLon{} // infinite intersections
	}
	if math.Sin(α1)*math.Sin(α2) < 0 {
		return LatLon{} // ambiguous intersection
	}

	α3 := math.Acos(-math.Cos(α1)*math.Cos(α2) + math.Sin(α1)*math.Sin(α2)*math.Cos(δ12))
	δ13 := math.Atan2(math.Sin(δ12)*math.Sin(α1)*math.Sin(α2), math.Cos(α2)+math.Cos(α1)*math.Cos(α3))
	φ3 := math.Asin(math.Sin(φ1)*math.Cos(δ13) + math.Cos(φ1)*math.Sin(δ13)*math.Cos(θ13))
	Δλ13 := math.Atan2(math.Sin(θ13)*math.Sin(δ13)*math.Cos(φ1), math.Cos(δ13)-math.Sin(φ1)*math.Sin(φ3))
	λ3 := λ1 + Δλ13

	return LatLon{mathx.Deg(φ3), math.Mod((mathx.Deg(λ3)+540), 360) - 180} // normalise to −180..+180°
}

// CrossTrackDistanceTo returns (signed) distance from ‘this’ point to great circle defined by start-point and end-point.
func (l LatLon) CrossTrackDistanceTo(pathStart LatLon, pathEnd LatLon, radius float64) float64 {
	var R float64
	if radius == 0 {
		R = 6371e3
	} else {
		R = radius
	}

	δ13 := pathStart.DistanceTo(l, R) / R
	θ13 := mathx.Rad(pathStart.BearingTo(l))
	θ12 := mathx.Rad(pathStart.BearingTo(pathEnd))

	δxt := math.Asin(math.Sin(δ13) * math.Sin(θ13-θ12))

	return δxt * R
}

// AlongTrackDistanceTo returns how far ‘this’ point is along a path from from start-point, heading towards end-point.
// That is, if a perpendicular is drawn from ‘this’ point to the (great circle) path, the along-track
// distance is the distance from the start point to where the perpendicular crosses the path.
func (l LatLon) AlongTrackDistanceTo(pathStart LatLon, pathEnd LatLon, radius float64) float64 {
	var R float64
	if radius == 0 {
		R = 6371e3
	} else {
		R = radius
	}

	δ13 := pathStart.DistanceTo(l, R) / R
	θ13 := mathx.Rad(pathStart.BearingTo(l))
	θ12 := mathx.Rad(pathStart.BearingTo(pathEnd))

	δxt := math.Asin(math.Sin(δ13) * math.Sin(θ13-θ12))

	δat := math.Acos(math.Cos(δ13) / math.Abs(math.Cos(δxt)))

	return δat * float64(mathx.Sign(math.Cos(θ12-θ13))) * R
}

// MaxLatitude returns maximum latitude reached when travelling on a great circle on given bearing from this
// point ('Clairaut's formula'). Negate the result for the minimum latitude (in the Southern
// hemisphere).
func (l LatLon) MaxLatitude(bearing float64) float64 {
	θ := mathx.Rad(bearing)

	φ := mathx.Rad(l.Lat)

	φMax := math.Acos(math.Abs(math.Sin(θ) * math.Cos(φ)))

	return mathx.Deg(φMax)
}

// CrossingParallels returns the pair of meridians at which a great circle defined by two points crosses the given
// latitude. If the great circle doesn't reach the given latitude, null is returned.
func (l LatLon) CrossingParallels(point1 LatLon, point2 LatLon, latitude float64) LatLon {
	φ := mathx.Rad(latitude)

	φ1 := mathx.Rad(point1.Lat)
	λ1 := mathx.Rad(point1.Lon)
	φ2 := mathx.Rad(point2.Lat)
	λ2 := mathx.Rad(point2.Lon)

	Δλ := λ2 - λ1

	x := math.Sin(φ1) * math.Cos(φ2) * math.Cos(φ) * math.Sin(Δλ)
	y := math.Sin(φ1)*math.Cos(φ2)*math.Cos(φ)*math.Cos(Δλ) - math.Cos(φ1)*math.Sin(φ2)*math.Cos(φ)
	z := math.Cos(φ1) * math.Cos(φ2) * math.Sin(φ) * math.Sin(Δλ)

	if z*z > x*x+y*y {
		return LatLon{} // great circle doesn't reach latitude
	}

	λm := math.Atan2(-y, x)                  // longitude at max latitude
	Δλi := math.Acos(z / math.Sqrt(x*x+y*y)) // Δλ from λm to intersection points

	λi1 := λ1 + λm - Δλi
	λi2 := λ1 + λm + Δλi

	return LatLon{math.Mod((mathx.Deg(λi1)+540), 360) - 180, math.Mod((mathx.Deg(λi2)+540), 360) - 180} // normalise to −180..+180°
}

// RhumbDistanceTo returns the distance travelling from ‘this’ point to destination point along a rhumb line.
func (l LatLon) RhumbDistanceTo(point LatLon, radius float64) float64 {
	if radius == 0 {
		radius = 6371e3
	}

	// see www.edwilliams.org/avform.htm#Rhumb

	R := radius
	φ1 := mathx.Rad(l.Lat)
	φ2 := mathx.Rad(point.Lat)
	Δφ := φ2 - φ1
	Δλ := mathx.Rad(math.Abs(point.Lon - l.Lon))
	// if dLon over 180° take shorter rhumb line across the anti-meridian:
	if Δλ > math.Pi {
		Δλ -= 2 * math.Pi
	}

	// on Mercator projection, longitude distances shrink by latitude; q is the 'stretch factor'
	// q becomes ill-conditioned along E-W line (0/0); use empirical tolerance to avoid it
	Δψ := math.Log(math.Tan(φ2/2+math.Pi/4) / math.Tan(φ1/2+math.Pi/4))
	var q float64
	if math.Abs(Δψ) > 10e-12 {
		q = Δφ / Δψ
	} else {
		q = math.Cos(φ1)
	}

	// distance is pythagoras on 'stretched' Mercator projection
	δ := math.Sqrt(Δφ*Δφ + q*q*Δλ*Δλ) // angular distance in radians
	dist := δ * R

	return dist
}

// RhumbBearingTo returns the bearing from ‘this’ point to destination point along a rhumb line.
func (l LatLon) RhumbBearingTo(point LatLon) float64 {
	φ1 := mathx.Rad(l.Lat)
	φ2 := mathx.Rad(point.Lat)
	Δλ := mathx.Rad(point.Lon - l.Lon)
	// if dLon over 180° take shorter rhumb line across the anti-meridian:
	if Δλ > math.Pi {
		Δλ -= 2 * math.Pi
	}

	if Δλ < -math.Pi {
		Δλ += 2 * math.Pi
	}

	Δψ := math.Log(math.Tan(φ2/2+math.Pi/4) / math.Tan(φ1/2+math.Pi/4))

	θ := math.Atan2(Δλ, Δψ)

	return math.Mod((mathx.Deg(θ) + 360), 360)
}

// RhumbDestinationPoint returns the destination point having travelled along a rhumb line from ‘this’ point the given
// distance on the  given bearing.
func (l LatLon) RhumbDestinationPoint(distance float64, bearing float64, radius float64) LatLon {
	if radius == 0 {
		radius = 6371e3
	}

	δ := distance / radius // angular distance in radians
	φ1 := mathx.Rad(l.Lat)
	λ1 := mathx.Rad(l.Lon)
	θ := mathx.Rad(bearing)

	Δφ := δ * math.Cos(θ)
	φ2 := φ1 + Δφ

	// check for some daft bugger going past the pole, normalise latitude if so
	if math.Abs(φ2) > math.Pi/2 {
		if φ2 > 0 {
			φ2 = math.Pi - φ2
		} else {
			φ2 = -math.Pi - φ2
		}
	}

	Δψ := math.Log(math.Tan(φ2/2+math.Pi/4) / math.Tan(φ1/2+math.Pi/4))
	var q float64
	if math.Abs(Δψ) > 10e-12 {
		q = Δφ / Δψ
	} else {
		q = math.Cos(φ1)
	} // E-W course becomes ill-conditioned with 0/0

	Δλ := δ * math.Sin(θ) / q
	λ2 := λ1 + Δλ

	return LatLon{mathx.Deg(φ2), math.Mod((mathx.Deg(λ2)+540), 360) - 180} // normalise to −180..+180°
}

// RhumbMidpointTo returns the loxodromic midpoint (along a rhumb line) between ‘this’ point and second point.
func (l LatLon) RhumbMidpointTo(point LatLon) LatLon {
	// see mathforum.org/kb/message.jspa?messageID=148837

	φ1 := mathx.Rad(l.Lat)
	λ1 := mathx.Rad(l.Lon)
	φ2 := mathx.Rad(point.Lat)
	λ2 := mathx.Rad(point.Lon)

	if math.Abs(λ2-λ1) > math.Pi {
		λ1 += 2 * math.Pi // crossing anti-meridian
	}

	φ3 := (φ1 + φ2) / 2
	f1 := math.Tan(math.Pi/4 + φ1/2)
	f2 := math.Tan(math.Pi/4 + φ2/2)
	f3 := math.Tan(math.Pi/4 + φ3/2)
	λ3 := ((λ2-λ1)*math.Log(f3) + λ1*math.Log(f2) - λ2*math.Log(f1)) / math.Log(f2/f1)

	if !math.IsInf(λ3, 0) {
		λ3 = (λ1 + λ2) / 2 // parallel of latitude
	}

	return LatLon{mathx.Deg(φ3), math.Mod((mathx.Deg(λ3)+540), 360) - 180} // normalise to −180..+180°
}

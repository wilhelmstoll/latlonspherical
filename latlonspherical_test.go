// Copyright 2019 Wilhelm Stoll. All rights reserved.
// Use of this source code is governed by a MIT-style
// license that can be found in the LICENSE file.

// Reference values which used in this test, were calculated with
// JavaScript originals by I{(C) Chris Veness 2011-2016}
// U{Latitude/Longitude<http://www.Movable-Type.co.UK/scripts/latlong.html>}.

package latlonspherical

import (
	"testing"

	"github.com/wilhelmstoll/testx"
)

// Vienna represents the testlocation.
var vienna = LatLon{48.210033, 16.363449}

// Graz represents the destination.
var graz = LatLon{47.076668, 15.421371}

func TestDistanceTo(t *testing.T) {
	distance := vienna.DistanceTo(graz, 0)

	testx.EqualFloat(t, 144438.6917821072, distance, "Distance")
}

func TestBearingTo(t *testing.T) {
	distance := vienna.BearingTo(graz)

	testx.EqualFloat(t, 209.59920480692168, distance, "Distance")
}

func TestFinalBearingTo(t *testing.T) {
	distance := vienna.FinalBearingTo(graz)

	testx.EqualFloat(t, 208.90300067306123, distance, "Distance")
}

func TestMidpointTo(t *testing.T) {
	point := vienna.MidpointTo(graz)
	testx.EqualFloat(t, 47.64431440188467, point.Lat, "Latitude")
	testx.EqualFloat(t, 15.88729992279184, point.Lon, "Longitude")
}

func TestIntermediatePointTo(t *testing.T) {
	// TODO
}

func TestDestinationPoint(t *testing.T) {
	point := vienna.DestinationPoint(10e3, 90.0, 0)

	testx.EqualFloat(t, 48.20995403358678, point.Lat, "Latitude")
	testx.EqualFloat(t, 16.498400694115617, point.Lon, "Longitude")
}

func TestIntersection(t *testing.T) {
	// TODO
}

func TestCrossTrackDistanceTo(t *testing.T) {
	// TODO
}

func TestAlongTrackDistanceTo(t *testing.T) {
	// TODO
}

func TestMaxLatitude(t *testing.T) {
	// TODO
}

func TestCrossingParallels(t *testing.T) {
	// TODO
}

func TestRhumbDistanceTo(t *testing.T) {
	// TODO
}

func TestRhumbBearingTo(t *testing.T) {
	// TODO
}

func TestRhumbDestinationPoint(t *testing.T) {
	// TODO
}

func TestRhumbMidpointTo(t *testing.T) {
	// TODO
}

# latlonspherical

Pure go implementation of geodetic (lat-/longitude) functions, transcribed in part from JavaScript (MIT Licence) originals by Chris Veness (http://www.Movable-Type.co.UK/scripts/latlong.html).

## Installation

```
go get -u github.com/wilhelmstoll/latlonspherical
```

## Example of usage

```go
package main

import (
	"fmt"

	"github.com/wilhelmstoll/latlonspherical"
)

func main() {
	vienna := latlonspherical.LatLon{Lat: 48.210033, Lon: 16.363449}
	graz := latlonspherical.LatLon{Lat: 47.076668, Lon: 15.421371}

	distance := vienna.DistanceTo(graz, 0)

	fmt.Println(distance)
}
```

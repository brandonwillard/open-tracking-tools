package org.opentrackingtools.util;

import org.opengis.referencing.operation.MathTransform;
import org.opengis.referencing.operation.NoninvertibleTransformException;
import org.opengis.referencing.operation.TransformException;
import org.testng.AssertJUnit;
import org.testng.annotations.Test;
import org.testng.annotations.DataProvider;

import com.vividsolutions.jts.geom.Coordinate;

public class GeoUtilsTest {

  @DataProvider
  public Object[][] coordinateData() {
    return new Object[][] {
      new Object[] { new Coordinate(42.017, -87.675)},
      new Object[] { new Coordinate(40.756, -73.981)},
      new Object[] { new Coordinate(-28.78, -65.63)},
      new Object[] { new Coordinate(31.7, 35.0)},
      new Object[] { new Coordinate(11.4, 122.8)}
    };
  }

  @Test(dataProvider="coordinateData")
  public void convertToEuclidean(Coordinate latLng) throws NoninvertibleTransformException, TransformException {
    Coordinate xy = GeoUtils.convertToEuclidean(latLng);
    
    MathTransform transform = GeoUtils.getTransform(latLng);
    
    Coordinate latLngAgain = GeoUtils.convertToLatLon(transform, xy);
    
    AssertJUnit.assertArrayEquals(new double[] {latLng.x, latLng.y}, 
        new double[] {latLngAgain.x, latLngAgain.y}, 1e-4);
  }

}

package org.opentrackingtools.util;

import org.opengis.referencing.operation.MathTransform;
import org.opengis.referencing.operation.NoninvertibleTransformException;
import org.opengis.referencing.operation.TransformException;
import org.testng.annotations.DataProvider;
import org.testng.annotations.Test;
import org.testng.internal.junit.ArrayAsserts;

import com.vividsolutions.jts.geom.Coordinate;

public class GeoUtilsTest {

  @Test(dataProvider = "coordinateData")
  public void convertToEuclidean(Coordinate latLng)
      throws NoninvertibleTransformException, TransformException {
    final Coordinate xy = GeoUtils.convertToEuclidean(latLng);

    final MathTransform transform = GeoUtils.getTransform(latLng);

    final Coordinate latLngAgain =
        GeoUtils.convertToLatLon(transform, xy);

    ArrayAsserts.assertArrayEquals(
        new double[] { latLng.x, latLng.y }, new double[] {
            latLngAgain.x, latLngAgain.y }, 1e-4);
  }

  @DataProvider
  public Object[][] coordinateData() {
    return new Object[][] {
        new Object[] { new Coordinate(42.017, -87.675) },
        new Object[] { new Coordinate(40.756, -73.981) },
        new Object[] { new Coordinate(-28.78, -65.63) },
        new Object[] { new Coordinate(31.7, 35.0) },
        new Object[] { new Coordinate(11.4, 122.8) } };
  }

}

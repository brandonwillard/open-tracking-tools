package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrix;
import gov.sandia.cognition.math.matrix.mtj.decomposition.EigenDecompositionRightMTJ;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.Set;

import org.apache.log4j.Logger;
import org.geotools.geometry.jts.JTS;
import org.geotools.geometry.jts.JTSFactoryFinder;
import org.geotools.referencing.CRS;
import org.opengis.geometry.MismatchedDimensionException;
import org.opengis.referencing.FactoryException;
import org.opengis.referencing.crs.CRSAuthorityFactory;
import org.opengis.referencing.crs.CoordinateReferenceSystem;
import org.opengis.referencing.operation.MathTransform;
import org.opengis.referencing.operation.TransformException;
import org.opentripplanner.graph_builder.impl.map.StreetMatcher;
import org.opentripplanner.model.GraphBundle;
import org.opentripplanner.routing.core.TraverseMode;
import org.opentripplanner.routing.core.TraverseOptions;
import org.opentripplanner.routing.graph.Edge;
import org.opentripplanner.routing.graph.Graph;
import org.opentripplanner.routing.graph.Vertex;
import org.opentripplanner.routing.impl.GraphServiceImpl;
import org.opentripplanner.routing.impl.StreetVertexIndexServiceImpl;
import org.opentripplanner.routing.location.StreetLocation;

import au.com.bytecode.opencsv.CSVReader;

import com.google.common.base.Objects;
import com.google.common.collect.ImmutableList;
import com.google.common.collect.Sets;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.CoordinateSequence;
import com.vividsolutions.jts.geom.Geometry;

public class GraphTest {

  private static final org.apache.log4j.Logger log = Logger
      .getLogger(GraphTest.class);
  private static final double gVariance = 50d;
  private static final double aVariance = 25d;
  private static final long avgTimeDiff = 1;
  private static final double initialAngularRate = Math.PI / 2d;

  public static void main(String[] args) {
    System.setProperty("org.geotools.referencing.forceXY", "true");
    final GraphServiceImpl gs = new GraphServiceImpl();
    final GraphBundle bundle = new GraphBundle(new File(
        "src/main/resources/org/openplans/cebutaxi/"));

    gs.setBundle(bundle);
    gs.refreshGraph();
    final Graph graph = gs.getGraph();
    final StreetMatcher streetMatcher = new StreetMatcher(graph);

    final StreetVertexIndexServiceImpl indexService = new StreetVertexIndexServiceImpl(
        graph);
    indexService.setup();

    final TraverseOptions options = new TraverseOptions(TraverseMode.CAR);

    final SimpleDateFormat sdf = new SimpleDateFormat("F/d/y H:m:s");

    final Standard2DTrackingFilter filter = new Standard2DTrackingFilter(gVariance,
        aVariance, 0d, null);
    MultivariateGaussian belief = null;

    final CSVReader gps_reader;
    final FileWriter test_output;
    try {
      final String googleWebMercatorCode = "EPSG:4326";

      final String cartesianCode = "EPSG:4499";

      final CRSAuthorityFactory crsAuthorityFactory = CRS
          .getAuthorityFactory(true);

      final CoordinateReferenceSystem mapCRS = crsAuthorityFactory
          .createCoordinateReferenceSystem(googleWebMercatorCode);

      final CoordinateReferenceSystem dataCRS = crsAuthorityFactory
          .createCoordinateReferenceSystem(cartesianCode);

      final boolean lenient = true; // allow for some error due to different
                                    // datums
      final MathTransform transform = CRS.findMathTransform(mapCRS, dataCRS,
          lenient);

      test_output = new FileWriter(
          "src/main/resources/org/openplans/cebutaxi/test_data/test_output.txt");
      test_output
          .write("time,original_lat,original_lon,kfMean_lat,kfMean_lon,kfMajor_lat,kfMajor_lon,kfMinor_lat,kfMinor_lon,graph_segment_id\n");

      gps_reader = new CSVReader(
          // new
          // FileReader("src/main/resources/org/openplans/cebutaxi/test_data/Cebu-Taxi-GPS/Day4-Taxi-1410-2101.txt"),
          // '\t');
          new FileReader(
              "src/main/resources/org/openplans/cebutaxi/test_data/Cebu-Taxi-GPS/0726.csv"),
          ',');
      String[] nextLine;
      gps_reader.readNext();
      log.info("processing gps data");

      long prevTime = 0;

      Coordinate prevObsCoords = null;
      final Matrix O = Standard2DTrackingFilter.getObservationMatrix();

      while ((nextLine = gps_reader.readNext()) != null) {

        final StringBuilder sb = new StringBuilder();

        // Date datetime = sdf.parse(nextLine[0] + " " + nextLine[1]);
        final Date datetime = sdf.parse(nextLine[0]);
        final long timeDiff = prevTime == 0 ? 0
            : (datetime.getTime() - prevTime) / 1000;
        prevTime = datetime.getTime();

        log.info("processing record time " + datetime.toString());

        final double lat = Double.parseDouble(nextLine[2]);
        final double lon = Double.parseDouble(nextLine[3]);

        sb.append(datetime.getTime()).append(",");
        sb.append(lat).append(",").append(lon).append(",");

        /*
         * Transform gps observation to cartesian coordinates
         */
        final Coordinate obsCoords = new Coordinate(lon, lat);
        final Coordinate obsPoint = new Coordinate();
        JTS.transform(obsCoords, obsPoint, transform);

        final Vector xyPoint = VectorFactory.getDefault().createVector2D(
            obsPoint.x, obsPoint.y);

        /*
         * Update the motion filter
         */
        final DenseMatrix covar;
        final Vector infMean;

        belief = updateFilter(timeDiff, xyPoint, filter, belief);
        if (timeDiff > 0) {
          // filter.measure(belief, xyPoint);
          // filter.predict(belief);
          filter.update(belief, xyPoint);

          infMean = O.times(belief.getMean().clone());
          covar = (DenseMatrix) O.times(belief.getCovariance().times(
              O.transpose()));
        } else {
          covar = (DenseMatrix) O.times(belief.getCovariance().times(
              O.transpose()));
          infMean = O.times(belief.getMean());
        }

        final EigenDecompositionRightMTJ decomp = EigenDecompositionRightMTJ
            .create(covar);
        final Matrix Shalf = MatrixFactory.getDefault().createIdentity(2, 2);
        Shalf
            .setElement(0, 0, Math.sqrt(decomp.getEigenValue(0).getRealPart()));
        Shalf
            .setElement(1, 1, Math.sqrt(decomp.getEigenValue(1).getRealPart()));
        final Vector majorAxis = infMean.plus(decomp.getEigenVectorsRealPart()
            .times(Shalf).scale(1.98).getColumn(0));
        final Vector minorAxis = infMean.plus(decomp.getEigenVectorsRealPart()
            .times(Shalf).scale(1.98).getColumn(1));

        /*
         * Transform state mean position coordinates to lat, lon
         */
        final Coordinate kfMean = new Coordinate();
        JTS.transform(
            new Coordinate(infMean.getElement(0), infMean.getElement(1)),
            kfMean, transform.inverse());
        sb.append(kfMean.y).append(",").append(kfMean.x).append(",");
        final Coordinate kfMajor = new Coordinate();
        JTS.transform(
            new Coordinate(majorAxis.getElement(0), majorAxis.getElement(1)),
            kfMajor, transform.inverse());
        sb.append(kfMajor.y).append(",").append(kfMajor.x).append(",");
        final Coordinate kfMinor = new Coordinate();
        JTS.transform(
            new Coordinate(minorAxis.getElement(0), minorAxis.getElement(1)),
            kfMinor, transform.inverse());
        sb.append(kfMinor.y).append(",").append(kfMinor.x).append(",");

        log.info("filter belief=" + belief.toString());

        log.info("attempting snap to graph for point " + obsCoords.toString());
        /*
         * Snap to graph
         */
        final Vertex snappedVertex = indexService.getClosestVertex(obsCoords,
            null, options);
        if (snappedVertex != null && (snappedVertex instanceof StreetLocation)) {
          final StreetLocation snappedStreetLocation = (StreetLocation) snappedVertex;
          final double dist = snappedVertex.distance(obsCoords);

          log.info("distance to graph: " + dist);
          log.info("vertexLabel=" + snappedVertex.getLabel());
          log.info("streetLocationName=" + snappedStreetLocation.getName());

          // List<StreetEdge> edges =
          // Objects.firstNonNull(snappedStreetLocation.getSourceEdges(),
          // ImmutableList.<StreetEdge>of());

          final Set<Integer> ids = Sets.newHashSet();
          if (prevObsCoords != null && !prevObsCoords.equals2D(obsCoords)) {
            final CoordinateSequence movementSeq = JTSFactoryFinder
                .getGeometryFactory().getCoordinateSequenceFactory()
                .create(new Coordinate[] { prevObsCoords, obsCoords });
            final Geometry movementGeometry = JTSFactoryFinder
                .getGeometryFactory().createLineString(movementSeq);
            final List<Edge> minimumConnectingEdges = streetMatcher
                .match(movementGeometry);

            for (final Edge edge : Objects.firstNonNull(minimumConnectingEdges,
                ImmutableList.<Edge> of())) {
              final Integer edgeId = graph.getIdForEdge(edge);
              if (edgeId != null)
                ids.add(edgeId);
            }
          } else {

            for (final Edge edge : Objects.firstNonNull(
                snappedStreetLocation.getOutgoingStreetEdges(),
                ImmutableList.<Edge> of())) {
              final Integer edgeId = graph.getIdForEdge(edge);
              if (edgeId != null)
                ids.add(edgeId);
            }

          }
          sb.append("\"" + ids.toString() + "\"");
        } else {
          sb.append("NA");
        }

        prevObsCoords = obsCoords;
        test_output.write(sb.toString() + "\n");
      }
    } catch (final FileNotFoundException e) {
      e.printStackTrace();
    } catch (final IOException e) {
      e.printStackTrace();
    } catch (final ParseException e) {
      e.printStackTrace();
    } catch (final FactoryException e) {
      e.printStackTrace();
    } catch (final MismatchedDimensionException e) {
      e.printStackTrace();
    } catch (final TransformException e) {
      e.printStackTrace();
    }
  }

  private static MultivariateGaussian updateFilter(long timeDiff,
    Vector xyPoint, Standard2DTrackingFilter filter, MultivariateGaussian belief) {
    /*
     * Initialize or update the kalman filter
     */
    if (belief == null) {
      belief = filter.createInitialLearnedObject();
      belief.setMean(VectorFactory.getDefault()
          .copyArray(
              new double[] { xyPoint.getElement(0), 0d, xyPoint.getElement(1),
                  0d }));
    } else {

      /*
       * We need to update the time-dependent components of this linear system
       * when time differences are non-constant.
       */
      // log.info("timeDiff (s)=" + timeDiff);
      //
      // Matrix modelCovariance = createStateCovariance(timeDiff/60d);
      // filter.setModelCovariance(modelCovariance);
      //
      // Matrix Gct = createStateTransitionMatrix(belief, timeDiff/60d);
      // Matrix G = MatrixFactory.getDefault().createIdentity(5, 5);
      // G.setSubMatrix(0, 0, Gct);
      // filter.getModel().setA(G);

    }
    return belief;
  }

}

package inference;

import org.codehaus.jackson.map.annotate.JsonSerialize;

import com.vividsolutions.jts.geom.Geometry;

import api.OsmSegment;

public class OsmSegmentWithMean extends OsmSegment {

  private Double mean;

  public OsmSegmentWithMean(Integer i, Geometry g, String name, Double mean) {
    super(i, g, name);
    this.mean = mean;
  }

  @JsonSerialize
  public Double getMean() {
    return mean;
  }

}

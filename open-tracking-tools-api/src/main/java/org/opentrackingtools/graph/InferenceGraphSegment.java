package org.opentrackingtools.graph;

import org.apache.commons.lang3.builder.ToStringBuilder;
import org.apache.commons.lang3.builder.ToStringStyle;

import javax.media.jai.IntegerSequence;

import com.google.common.base.Preconditions;
import com.google.common.collect.ComparisonChain;
import com.google.common.primitives.Doubles;
import com.google.common.primitives.Ints;
import com.vividsolutions.jts.geom.LineSegment;
import com.vividsolutions.jts.linearref.LinearLocation;

public class InferenceGraphSegment extends InferenceGraphEdge {

  final protected LinearLocation endIndex;
  final protected LineSegment line;
  final protected InferenceGraphSegment nextSegment;
  final protected Double startDistance;
  final protected LinearLocation startIndex;
  
  final public static InferenceGraphSegment nullGraphSegment = new InferenceGraphSegment();

  protected InferenceGraphSegment() {
    super();
    this.endIndex = null;
    this.startIndex = null;
    this.startDistance = null;
    this.nextSegment = null;
    this.line = null;
  }
  
  public InferenceGraphSegment(LineSegment line,
    InferenceGraphEdge infEdge, InferenceGraphSegment nextSegment) {
    super(infEdge);
    this.line = line;
    this.startIndex =
        infEdge.getLocationIndexedLine().indexOf(line.p0);
    this.endIndex =
        infEdge.getLocationIndexedLine().indexOfAfter(line.p1,
            this.startIndex);
    this.startDistance =
        infEdge.getLengthLocationMap().getLength(this.startIndex);
    this.nextSegment = nextSegment;
  }

  @Override
  public int compareTo(InferenceGraphEdge o) {
    ComparisonChain chain = ComparisonChain.start()
        .compare(this.edgeId, o.edgeId);
    
    if (o instanceof InferenceGraphSegment)
      chain = chain.compare(this.startIndex, ((InferenceGraphSegment)o).startIndex);
    
    return chain.result();
  }

  public LinearLocation getEndIndex() {
    return this.endIndex;
  }

  public LineSegment getLine() {
    return this.line;
  }

  public InferenceGraphSegment getNextSegment() {
    return this.nextSegment;
  }

  public double getStartDistance() {
    return this.startDistance;
  }

  public LinearLocation getStartIndex() {
    return this.startIndex;
  }

  @Override
  public String toString() {
    final ToStringBuilder builder = new ToStringBuilder(this);
    builder.append("parentEdge", this.getEdgeId());
    if (!this.isNullEdge()) {
      builder.append("startIndex", this.startIndex.getSegmentIndex());
      builder.append("line", this.line);
    }
    return builder.toString();
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = super.hashCode();
    
    result = prime * result
        + ((startIndex != null) ? Doubles.hashCode(startIndex.getSegmentFraction()) : 0);
    result = prime * result
        + ((startIndex != null) ? Ints.hashCode(startIndex.getComponentIndex()) : 0);
    result = prime * result
        + ((startIndex != null) ? Ints.hashCode(startIndex.getSegmentIndex()) : 0);
    return result;
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj) {
      return true;
    }
    if (!super.equals(obj)) {
      return false;
    }
    Preconditions.checkArgument(obj instanceof InferenceGraphSegment);
//    if (!(obj instanceof InferenceGraphSegment)) {
//      return false;
//    }
    InferenceGraphSegment other = (InferenceGraphSegment) obj;
    if (startIndex == null) {
      if (other.startIndex != null) {
        return false;
      }
    } else if (startIndex.compareTo(other.startIndex) != 0) {
      return false;
    }
    return true;
  }

  @Override
  public boolean isNullEdge() {
    return this.equals(InferenceGraphSegment.nullGraphSegment);
  }

}

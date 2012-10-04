package models;

import java.math.BigInteger;
import java.util.List;

import javax.persistence.Entity;
import javax.persistence.EntityManager;
import javax.persistence.Query;

import org.apache.commons.lang.StringUtils;
import org.hibernate.annotations.Type;
import org.openplans.tools.tracking.impl.ObservationData;

import com.vividsolutions.jts.geom.LineString;

import play.db.jpa.Model;

@Entity
public class StreetEdge extends Model {
 
    public Integer edgeId;
    
    @Type(type = "org.hibernatespatial.GeometryUserType")
    public LineString shape;
    
    public Integer inPath;
    
    public Double meanVelocity;
    public Double velocityVarience;
    
    public Double v0;
    public Double v1;
    public Double v2;
    public Double v3;
    public Double v4;
    public Double v5;
    public Double v6;
    public Double v7;
    public Double v8;
    public Double v9;
    public Double v10;
    public Double v11;
    public Double v12;
    public Double v13;
    public Double v14;
    public Double v15;
    public Double v16;
    public Double v17;
    public Double v18;
    public Double v19;
    public Double v20;
    public Double v21;
    public Double v22;
    public Double v23;
    
    public Double c0;
    public Double c1;
    public Double c2;
    public Double c3;
    public Double c4;
    public Double c5;
    public Double c6;
    public Double c7;
    public Double c8;
    public Double c9;
    public Double c10;
    public Double c11;
    public Double c12;
    public Double c13;
    public Double c14;
    public Double c15;
    public Double c16;
    public Double c17;
    public Double c18;
    public Double c19;
    public Double c20;
    public Double c21;
    public Double c22;
    public Double c23;
    
    
    public String rbgColor;
    
    
    static public void updatePathEdges(EntityManager em, List<Integer> edges)
    {
    	
    	String edgesIds = StringUtils.join(edges, ", "); 
    	em.getTransaction().begin();
    	
    	String sql = "UPDATE streetedge SET inPath = 1 WHERE edgeid IN (" + edgesIds + ");";
    	
    	em.createNativeQuery(sql)
    			.executeUpdate();
    	
    	em.getTransaction().commit();
    }
}


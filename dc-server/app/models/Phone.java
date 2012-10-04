package models;

import java.util.Date;

import javax.persistence.Entity;
import javax.persistence.ManyToOne;

import play.db.jpa.Model;

@Entity
public class Phone extends Model {

	public String imei;
	
	@ManyToOne
	public Operator operator;
	
	@ManyToOne
	public Driver driver;
	
	@ManyToOne
	public Vehicle vehicle;
	
	public Date lastUpdate;
	    
    public Double recentLat;
    public Double recentLon;
}
 
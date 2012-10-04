package controllers;

import play.*;
import play.mvc.*;

import inference.InferenceStore;
import inference.ObservationActor;

import java.io.File;
import java.util.*;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicLong;
import java.lang.System;

import org.openplans.tools.tracking.impl.util.OtpGraph;

import akka.actor.ActorRef;
import akka.actor.ActorSystem;
import akka.actor.Cancellable;
import akka.actor.Props;
import akka.util.Duration;

import com.typesafe.config.Config;
import com.typesafe.config.ConfigFactory;

import models.*;


public class Application extends Controller {

	
	// otp graph
	
	public static OtpGraph graph = new OtpGraph(
		      Play.configuration.getProperty("application.otpGraphPath"), null);
	
	public static OtpGraph getGraph() {
		return graph;
	}
	
	// statistics related data 
	
	static final Long STATS_FREQUENCY = new Long(10); // seconds between stats updates 
	
	static Long lastClock = new Long(0);
	
	static ConcurrentLinkedQueue<Double> timeStats = new ConcurrentLinkedQueue<Double>();
	
	
	// akka setup 
	
	static Config inferenceConfig = ConfigFactory.parseFile(new File("conf/akka.conf"));
	
	static ActorSystem system = ActorSystem.create("inferenceSystem", inferenceConfig);

	static ActorRef observationActor = system.actorOf(new Props(ObservationActor.class).withDispatcher("my-thread-pool-dispatcher"), "observationActor");

	
	// statistic monitor
	
	static Cancellable timeUpdate = system.scheduler().schedule(Duration.create(STATS_FREQUENCY, TimeUnit.SECONDS), Duration.create(STATS_FREQUENCY, TimeUnit.SECONDS), new Runnable() {
		  @Override
		  public void run() {
		  
			synchronized (this)
			{
				Long currentClock = System.nanoTime();
				
				Long messages = ObservationActor.messageCount.getAndSet(0);
				
				Double messageRate = (double)messages / ((double)(currentClock - lastClock) / 1000000000.0 );
				
				timeStats.add(messageRate);
				
				Logger.info("Message rate: " + messageRate);	
			
				lastClock = currentClock;
			}	  			
		  }
	});
	
	
    public static void index() {
  
    	
        render();
    }
    
    public static void reset() {
  
    	InferenceStore.removeAll();
    	
        index();
    }
    
    public static void getBestPath(String vehicleId) {
     	
        renderJSON(InferenceStore.getInferenceInstance(vehicleId).getBestState().getPath().getEdgeIds());
    }

}
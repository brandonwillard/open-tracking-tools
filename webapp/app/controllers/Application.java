package controllers;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import inference.InferenceService;
import inference.Simulation;
import inference.Simulation.SimulationActor;

import java.io.File;
import java.io.FileOutputStream;
import java.util.List;

import org.apache.commons.io.IOUtils;
import org.openplans.tools.tracking.impl.VehicleState.InitialParameters;

import models.InferenceInstance;

import akka.actor.ActorRef;
import akka.actor.ActorSystem;
import akka.actor.Props;
import async.CsvUploadActor;

import play.*;
import play.mvc.*;

public class Application extends Controller {
	
  static ActorSystem system = ActorSystem.create("MySystem");
  static ActorRef simActor = system.actorOf(new Props(SimulationActor.class), "simActor");
  static ActorRef locationActor = system.actorOf(new Props(InferenceService.class), "locationActor");
  static ActorRef csvActor = system.actorOf(new Props(CsvUploadActor.class), "csvActor");
  

  public static void map(String name) {
	  render(name);
  }
  

  public static void getInferenceInstances() {
	  List<InferenceInstance> instances = InferenceService.getInferenceInstances();
	  render(instances);
  }
  
  public static void removeInferenceInstance(String name) {
    InferenceService.remove(name);
  	instances();     
  }
  
  public static void simulation(String obs_variance_pair, String road_state_variance_pair,
    String ground_state_variance_pair, String off_prob_pair, String on_prob_pair, 
    String performInference) {
    
    final String[] obsPair = obs_variance_pair.split(",");
    final Vector obsVariance = VectorFactory.getDefault().createVector2D(
        Double.parseDouble(obsPair[0]), Double.parseDouble(obsPair[1]));
    
    final String[] roadStatePair = road_state_variance_pair.split(",");
    final Vector roadStateVariance = VectorFactory.getDefault().createVector2D(
        Double.parseDouble(roadStatePair[0]), Double.parseDouble(roadStatePair[1]));
    
    final String[] groundStatePair = ground_state_variance_pair.split(",");
    final Vector groundStateVariance = VectorFactory.getDefault().createVector2D(
        Double.parseDouble(groundStatePair[0]), Double.parseDouble(groundStatePair[1]));
    
    final String[] offPair = off_prob_pair.split(",");
    final Vector offProbs = VectorFactory.getDefault().createVector2D(
        Double.parseDouble(offPair[0]), Double.parseDouble(offPair[1]));
    
    final String[] onPair = on_prob_pair.split(",");
    final Vector onProbs = VectorFactory.getDefault().createVector2D(
        Double.parseDouble(onPair[0]), Double.parseDouble(onPair[1]));
    
    final boolean inference = Boolean.parseBoolean(performInference);
    
    InitialParameters parameters = new InitialParameters(obsVariance, roadStateVariance, 
        groundStateVariance, offProbs, onProbs);
    
    Simulation sim = new Simulation(parameters, inference);
    Logger.info("starting simulation " + sim.getSimulationName());

    if (InferenceService.getInferenceInstance(sim.getSimulationName(), true) != null) {
      Logger.warn("removing existing inference instance = " + sim.getSimulationName());
      InferenceService.remove(sim.getSimulationName());
    }
    
    Application.simActor.tell(sim);
    
  	instances();     
  }
  
  public static void instances() {
	  List<InferenceInstance> instances = InferenceService.getInferenceInstances();
	  render(instances);
  }
  
  public static void uploadHandler(File csv) {
	  
	  if (csv != null) {
		File dest = new File("/tmp/upload.csv");
		csv.renameTo(dest);
	    csvActor.tell(dest);
	  }
	 
	  instances();     
  }  
  

 
  
}
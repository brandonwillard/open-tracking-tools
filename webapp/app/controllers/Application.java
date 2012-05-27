package controllers;

import inference.InferenceService;
import inference.Simulation;
import inference.Simulation.SimulationActor;

import java.io.File;
import java.io.FileOutputStream;
import java.util.List;

import org.apache.commons.io.IOUtils;

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
  
  public static void simulation() {
    Simulation sim = new Simulation();
    Logger.info("starting simulation " + sim.getSimulationName());

    if (InferenceService.getInferenceInstance(sim.getSimulationName()) != null) {
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
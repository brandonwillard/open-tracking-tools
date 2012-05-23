package controllers;

import inference.InferenceService;
import inference.Simulation;

import java.io.File;

import models.InferenceInstance;

import akka.actor.ActorRef;
import akka.actor.Props;
import async.CsvUploadActor;

import play.*;
import play.data.*;
import play.libs.Akka;
import play.mvc.*;
import play.mvc.Http.MultipartFormData;
import play.mvc.Http.MultipartFormData.FilePart;

import views.html.*;

public class Application extends Controller {
  
  public static Result map() {
    return ok(map.render(null));
  }
  
  public static Result map(String name) {
    return ok(map.render(name));
  }
  
  static Form<InferenceInstance> inferenceInstanceForm = form(InferenceInstance.class);
  
  public static Result getInferenceInstances() {
	  return ok(instances.render(InferenceService.getInferenceInstances(), inferenceInstanceForm));
  }
  
  public static Result removeInferenceInstance(String name) {
    InferenceService.remove(name);
	  return redirect(routes.Application.instances());     
  }
  
  public static Result simulation() {
    Simulation sim = new Simulation();
    Logger.info("starting simulation " + sim.getSimulationName());

    InferenceService.addSimulationRecords(sim.getSimulationName(), sim.runSimulation());
    
	  return redirect(routes.Application.instances());     
  }
  
  public static Result instances() {
	  return ok(instances.render(InferenceService.getInferenceInstances(), inferenceInstanceForm));
  }
  
  public static Result uploadHandler() {
	  MultipartFormData body = request().body().asMultipartFormData();
	  FilePart csv = body.getFile("csv");
	  
	  if (csv != null) {
	 
	    File csvFile = csv.getFile();
	    
	    ActorRef csvActor = Akka.system().actorOf(new Props(CsvUploadActor.class));
	    
	    csvActor.tell(csvFile);
	  }
	 
	  return redirect(routes.Application.instances());     
  }  
  

 
  
}
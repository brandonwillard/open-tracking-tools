package controllers;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import inference.InferenceService;
import inference.Simulation;
import inference.Simulation.SimulationActor;
import inference.Simulation.SimulationParameters;

import java.io.File;
import java.util.Date;
import java.util.List;

import models.InferenceInstance;

import org.openplans.tools.tracking.impl.VehicleState.InitialParameters;

import play.Logger;
import play.mvc.Controller;
import akka.actor.ActorRef;
import akka.actor.ActorSystem;
import akka.actor.Props;
import async.CsvUploadActor;
import async.CsvUploadActor.TraceParameters;

import com.vividsolutions.jts.geom.Coordinate;

public class Application extends Controller {

  static ActorSystem system = ActorSystem.create("MySystem");
  static ActorRef simActor = system.actorOf(new Props(
      SimulationActor.class), "simActor");
  static ActorRef locationActor = system.actorOf(new Props(
      InferenceService.class), "locationActor");
  static ActorRef csvActor = system.actorOf(new Props(
      CsvUploadActor.class), "csvActor");

  public static void getInferenceInstances() {
    final List<InferenceInstance> instances = InferenceService
        .getInferenceInstances();
    render(instances);
  }

  public static void instances() {
    final List<InferenceInstance> instances = InferenceService
        .getInferenceInstances();
    render(instances);
  }

  public static void map(String name) {
    render(name);
  }

  public static void removeInferenceInstance(String name) {
    InferenceService.remove(name);
    instances();
  }

  public static void simulation(String obs_variance_pair,
    String road_state_variance_pair,
    String ground_state_variance_pair, String off_prob_pair,
    String on_prob_pair, String performInference,
    String start_coordinate_pair, String start_unix_time,
    String duration_str, String frequency_str, String seed_str) {

    final String[] startCoordPair = start_coordinate_pair.split(",");
    final Coordinate startCoord = new Coordinate(
        Double.parseDouble(startCoordPair[0]),
        Double.parseDouble(startCoordPair[1]));
    final Date startTime = new Date(Long.parseLong(start_unix_time));
    final long seed = Long.parseLong(seed_str);
    final long duration = Long.parseLong(duration_str);
    final long frequency = Long.parseLong(frequency_str);
    final boolean inference = Boolean.parseBoolean(performInference);
    final SimulationParameters simParams = new SimulationParameters(
        startCoord, startTime, duration, frequency, inference);

    final String[] obsPair = obs_variance_pair.split(",");
    final Vector obsVariance = VectorFactory.getDefault()
        .createVector2D(
            Double.parseDouble(obsPair[0]),
            Double.parseDouble(obsPair[1]));

    final String[] roadStatePair = road_state_variance_pair
        .split(",");
    final Vector roadStateVariance = VectorFactory.getDefault()
        .createVector2D(
            Double.parseDouble(roadStatePair[0]),
            Double.parseDouble(roadStatePair[1]));

    final String[] groundStatePair = ground_state_variance_pair
        .split(",");
    final Vector groundStateVariance = VectorFactory.getDefault()
        .createVector2D(
            Double.parseDouble(groundStatePair[0]),
            Double.parseDouble(groundStatePair[1]));

    final String[] offPair = off_prob_pair.split(",");
    final Vector offProbs = VectorFactory.getDefault()
        .createVector2D(
            Double.parseDouble(offPair[0]),
            Double.parseDouble(offPair[1]));

    final String[] onPair = on_prob_pair.split(",");
    final Vector onProbs = VectorFactory.getDefault().createVector2D(
        Double.parseDouble(onPair[0]), Double.parseDouble(onPair[1]));

    final InitialParameters parameters = new InitialParameters(
        obsVariance, roadStateVariance, groundStateVariance,
        offProbs, onProbs, seed);

    final String simulationName = "sim-" + start_unix_time;
    if (InferenceService.getInferenceInstance(simulationName) != null) {
      Logger.warn("removing existing inference instance named "
          + simulationName);
      InferenceService.remove(simulationName);
    }
    final Simulation sim = new Simulation(
        simulationName, parameters, simParams);
    Logger.info("starting simulation " + sim.getSimulationName());

    Application.simActor.tell(sim);

    instances();
  }

  public static void uploadHandler(File csv, String debugEnabled) {

    if (csv != null) {
      final boolean debug_enabled = Boolean
          .parseBoolean(debugEnabled);
      final File dest = new File("/tmp/upload.csv");
      csv.renameTo(dest);
      final TraceParameters params = new TraceParameters(
          dest, debug_enabled);
      csvActor.tell(params);
    }

    instances();
  }

}
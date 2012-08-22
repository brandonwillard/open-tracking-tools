library("rjson")

getSimJson <- function(simName) {
  json_file <-
    paste("http://localhost:9000/api/getPerformanceResults?vehicleId=", simName,
          sep="")

  json_data <- readLines(json_file)
  json_data <- sub('[^\\{]*', '', json_data) # remove function name and opening parenthesis
  json_data <- sub('\\)$', '', json_data) # remove closing parenthesis

  json_data <- fromJSON(paste(json_data, collapse=""))
  return(json_data$results)
}

# avg. records per sec = 67.35140596059942
pl_means_25_s1 <- unlist(lapply(getSimJson("sim-582243527"), function(x) sqrt(x$mean)))

# avg. records per sec = 39.146604032100214  
pl_means_50_s1 <- unlist(lapply(getSimJson("sim130512018"), function(x) sqrt(x$mean)))

# avg. records per sec = 11.59386684443929  
pl_means_200_s1 <- unlist(lapply(getSimJson("sim112077992"), function(x) sqrt(x$mean)))

# avg. records per sec = 5.587866878384734 
#pl_means_500_s1 <- unlist(lapply(getSimJson("sim75209940"), function(x) sqrt(x$mean)))

# avg. records per sec = 1673.6401673640166
bs_means_50_s1 <- unlist(lapply(getSimJson("sim1108672430"), function(x) sqrt(x$mean))) 

# avg. records per sec = 215.74973031283707
bs_means_200_s1 <- unlist(lapply(getSimJson("sim1090238404"), function(x) sqrt(x$mean)))

# avg. records per sec =51.49330587023687
bs_means_500_s1 <- unlist(lapply(getSimJson("sim1053370352"), function(x) sqrt(x$mean)))


library(ggplot2)
data = data.frame(
                  "Time"=rep(c(1:1200, 1:1200, 
                                1:1200), 2),
                  "Filter" = c(rep("PL", 3*1200), rep("BS", 3*1200)),
                  "Particles"=c(
                                c(rep("50", 1200), rep("200", 1200), rep("25", 1200)), 
                                c(rep("50", 1200), rep("200", 1200), rep("500", 1200))
                      ),
                  "RMSE"=c(pl_means_50_s1, pl_means_200_s1, pl_means_25_s1, 
                    bs_means_50_s1, bs_means_200_s1, bs_means_500_s1))
data$partFilter = interaction(data$Filter, data$Particles)

pdf(file="plbs-rmse-1200-wisker.pdf")
  #par(mfrow=c(1,1), las=0)
  #lim = 1200
  #pl_means_all = cbind("PL 50 particles"=pl_means_50_s1
  #           ,"PL 200 particles"=pl_means_200_s1
  #           ,"PL 25 particles"=pl_means_25_s1
  #            ,"Bootstrap 50 particles"=bs_means_50_s1
  #            ,"Bootstrap 200 particles"=bs_means_200_s1
  #            ,"Bootstrap 500 particles"=bs_means_500_s1
  #            )
  #boxplot(pl_means_all, log="y", ylab="RMSE")

  p2 <- ggplot(data, aes(partFilter, RMSE)) + geom_boxplot() + scale_y_log10() +
        theme_bw(base_size=10) + scale_x_discrete(name="")                            
  print(p2)

dev.off()


pdf(file="plbs-rmse-1200-series.pdf", pointsize=7) 
  p1 <- ggplot(data, aes(x=Time, y=RMSE, colour=Particles, group=Filter)) +
        geom_line(alpha=0.7) + facet_grid( . ~ Filter) + scale_y_log10() +
        theme_bw(base_size=10)
  plot(p1)

  #plot(cbind("PL 50 particles"=pl_means_50_s1
  #             ,"PL 200 particles"=pl_means_200_s1
  #             ,"PL 500 particles"=pl_means_500_s1
  #              ,"Bootstrap 50 particles"=bs_means_50_s1
  #              ,"Bootstrap 200 particles"=bs_means_200_s1
  #              ,"Bootstrap 500 particles"=bs_means_500_s1
  #              ), 
  #        lty=1,
  #        type="l", log="y", ylab="RMSE")
  #legend("topleft", c("PL 50 particles", 
  #                    "PL 200 particles",
  #                    "PL 500 particles"
  #                    "Bootstrap 50 particles", 
  #                    "Bootstrap 200 particles",
  #                    "Bootstrap 500 particles"
  #                    ),
  #       pch="l", col=c("black", "green", "red", "blue"))
dev.off()




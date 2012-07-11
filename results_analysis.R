library("rjson")
json_file <-
  "http://localhost:9000/api/getPerformanceResults?vehicleId=sim-1325570441000"

json_data <- readLines(json_file)
json_data <- sub('[^\\{]*', '', json_data) # remove function name and opening parenthesis
json_data <- sub('\\)$', '', json_data) # remove closing parenthesis

json_data <- fromJSON(paste(json_data, collapse=""))

means <- unlist(lapply(json_data$results, function(x) x$mean))
upper <- unlist(lapply(json_data$results, function(x) x$mean + 1.98*sqrt(x$variance)))

par(mfrow=c(2,1))
matplot(cbind(means, upper), type='l', ylab="MSE")
hist(means)

which.max(means)

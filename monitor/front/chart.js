var a = [0];
var seconds = 0;

function getNewSeries() {
    seconds++;
    document.querySelector("#time").innerHTML = (Math.ceil(seconds / 60)-1) + ":" + (seconds % 60); 
    axios.get("http://localhost:5000/data")
    .then(function (response) {
    // handle success
        document.querySelector("#x").innerHTML = "x: " + response.data["x"];
        document.querySelector("#y").innerHTML = "y: " + response.data["y"];
        document.querySelector("#angle").innerHTML = "angle: " + response.data["angle"];
        document.querySelector("#speed").innerHTML = "speed: " + response.data["speed"] + " mm/s";
        if(response.data["alert"]) {
            a.push(a[a.length-1] + 1);
        } else {
            a.push(a[a.length-1]);
        }
    })
    .catch(function (error) {
    // handle error
        console.log(error);
    })
    .then(function () {
    // always executed
    });
}

var options = {
    series: [{
    data: a
  }],
    chart: {
    id: 'realtime',
    height: 350,
    type: 'line',
    animations: {
      enabled: true,
      easing: 'linear',
      dynamicAnimation: {
        speed: 1000
      }
    },
    toolbar: {
      show: false
    },
    zoom: {
      enabled: false
    }
  },
  dataLabels: {
    enabled: false
  },
  stroke: {
    curve: 'smooth'
  },
  title: {
    text: 'Alert frequency',
    align: 'left'
  },
  markers: {
    size: 0
  },
  xaxis: {
    type: 'number',
    max: 200,
    range: 200,
    axisTicks: {
        color: 0xfff
    },
    labels: {
        show: false,
    },
  },
  yaxis: {
    max: 20,
    labels: {
        style: {
            colors: 0xfff,
        }
    },
  },
  legend: {
    show: false
  },
  };

  var chart = new ApexCharts(document.querySelector("#alert"), options);
  chart.render();


  window.setInterval(function () {
  getNewSeries();

  chart.updateSeries([{
    data: a
  }])
}, 1000);


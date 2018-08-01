function init_imu_listener(ctx) {
    var chart_imu_a = new Chart('imu_a', {
        type: 'line',
        data: {
            datasets: [{
                label: 'x',
                borderColor: 'rgba(255,99,132,1)',
                backgroundColor: 'rgba(255,255,255,0)',
                data: []
            }, {
                label: 'y',
                borderColor: 'rgba(54, 162, 235, 1)',
                backgroundColor: 'rgba(255,255,255,0)',
                data: []
            }, {
                label: 'z',
                borderColor: 'rgba(75, 192, 192, 1)',
                backgroundColor: 'rgba(255,255,255,0)',
                data: []
            }]
        },
        options: {
            elements: {
                line: {
                    tension: 0,
                }
            },
            scales: {
                xAxes: [{
                    type: 'realtime',
                    time: {
                        displayFormats: {
                            second: 'hh:mm:ss'
                        }
                    }
                }]
            },
            animation: {
                duration: 0
            },
            hover: {
                animationDuration: 0
            },
            responsiveAnimationDuration: 0,
            plugins: {
                streaming: {
                    frameRate: 30,
                    refresh: 500,
                    delay: 500,
                    duration: 10000,
                    onRefresh: function (chart) {
                        if (imu == null) {
                            return;
                        }
                        chart.data.datasets[0].data.push({ x: Date.now(), y: imu.linear_acceleration.x });
                        chart.data.datasets[1].data.push({ x: Date.now(), y: imu.linear_acceleration.y });
                        chart.data.datasets[2].data.push({ x: Date.now(), y: imu.linear_acceleration.z });
                    }
                }
            }
        }
    });

    var chart_imu_w = new Chart('imu_w', {
        type: 'line',
        data: {
            datasets: [{
                label: 'x',
                borderColor: 'rgba(255,99,132,1)',
                backgroundColor: 'rgba(255,255,255,0)',
                data: []
            }, {
                label: 'y',
                borderColor: 'rgba(54, 162, 235, 1)',
                backgroundColor: 'rgba(255,255,255,0)',
                data: []
            }, {
                label: 'z',
                borderColor: 'rgba(75, 192, 192, 1)',
                backgroundColor: 'rgba(255,255,255,0)',
                data: []
            }]
        },
        options: {
            elements: {
                line: {
                    tension: 0,
                }
            },
            scales: {
                xAxes: [{
                    type: 'realtime',
                    time: {
                        displayFormats: {
                            second: 'hh:mm:ss'
                        }
                    }
                }]
            },
            animation: {
                duration: 0
            },
            hover: {
                animationDuration: 0
            },
            responsiveAnimationDuration: 0,
            plugins: {
                streaming: {
                    frameRate: 30,
                    refresh: 500,
                    delay: 500,
                    duration: 10000,
                    onRefresh: function (chart) {
                        if (imu == null) {
                            return;
                        }
                        chart.data.datasets[0].data.push({ x: Date.now(), y: imu.angular_velocity.x });
                        chart.data.datasets[1].data.push({ x: Date.now(), y: imu.angular_velocity.y });
                        chart.data.datasets[2].data.push({ x: Date.now(), y: imu.angular_velocity.z });
                    }
                }
            }
        }
    });

    var chart_imu_e = new Chart('imu_e', {
        type: 'line',
        data: {
            datasets: [{
                label: 'roll',
                borderColor: 'rgba(255,99,132,1)',
                backgroundColor: 'rgba(255,255,255,0)',
                data: []
            }, {
                label: 'pitch',
                borderColor: 'rgba(54, 162, 235, 1)',
                backgroundColor: 'rgba(255,255,255,0)',
                data: []
            }, {
                label: 'yaw',
                borderColor: 'rgba(75, 192, 192, 1)',
                backgroundColor: 'rgba(255,255,255,0)',
                data: []
            }]
        },
        options: {
            elements: {
                line: {
                    tension: 0,
                }
            },
            scales: {
                xAxes: [{
                    type: 'realtime',
                    time: {
                        displayFormats: {
                            second: 'hh:mm:ss'
                        }
                    }
                }]
            },
            animation: {
                duration: 0
            },
            hover: {
                animationDuration: 0
            },
            responsiveAnimationDuration: 0,
            plugins: {
                streaming: {
                    frameRate: 30,
                    refresh: 500,
                    delay: 500,
                    duration: 10000,
                    onRefresh: function (chart) {
                        if (imu == null) {
                            return;
                        }

                        var [roll, pitch, yaw] = quaternion_to_euler_angles(imu.orientation);
                        chart.data.datasets[0].data.push({ x: Date.now(), y: roll + 90 });
                        chart.data.datasets[1].data.push({ x: Date.now(), y: pitch });
                        chart.data.datasets[2].data.push({ x: Date.now(), y: yaw });
                    }
                }
            }
        }
    });
}

function init_dvl_listener(ctx) {
    var chart_dvl = new Chart('dvl', {
        type: 'line',
        data: {
            datasets: [{
                label: 'x',
                borderColor: 'rgba(255,99,132,1)',
                backgroundColor: 'rgba(255,255,255,0)',
                data: []
            }, {
                label: 'y',
                borderColor: 'rgba(54, 162, 235, 1)',
                backgroundColor: 'rgba(255,255,255,0)',
                data: []
            }, {
                label: 'z',
                borderColor: 'rgba(75, 192, 192, 1)',
                backgroundColor: 'rgba(255,255,255,0)',
                data: []
            }]
        },
        options: {
            elements: {
                line: {
                    tension: 0,
                }
            },
            scales: {
                xAxes: [{
                    type: 'realtime',
                    time: {
                        displayFormats: {
                            second: 'hh:mm:ss'
                        }
                    }
                }]
            },
            animation: {
                duration: 0
            },
            hover: {
                animationDuration: 0
            },
            responsiveAnimationDuration: 0,
            plugins: {
                streaming: {
                    frameRate: 30,
                    refresh: 500,
                    delay: 500,
                    duration: 10000,
                    onRefresh: function (chart) {
                        if (dvl == null) {
                            return;
                        }
                        chart.data.datasets[0].data.push({ x: Date.now(), y: dvl.velocity.x });
                        chart.data.datasets[1].data.push({ x: Date.now(), y: dvl.velocity.y });
                        chart.data.datasets[2].data.push({ x: Date.now(), y: dvl.velocity.z });
                    }
                }
            }
        }
    });
}

function init_depth_listener(ctx) {
    var chart_depth = new Chart('depth', {
        type: 'line',
        data: {
            datasets: [{
                label: 'depth',
                borderColor: 'rgba(54, 162, 235, 1)',
                backgroundColor: 'rgba(255,255,255,0)',
                data: []
            }, {
                label: 'altitude',
                borderColor: 'rgba(75, 192, 192, 1)',
                backgroundColor: 'rgba(255,255,255,0)',
                data: []
            }]
        },
        options: {
            elements: {
                line: {
                    tension: 0,
                }
            },
            scales: {
                xAxes: [{
                    type: 'realtime',
                    time: {
                        displayFormats: {
                            second: 'hh:mm:ss'
                        }
                    }
                }]
            },
            animation: {
                duration: 0
            },
            hover: {
                animationDuration: 0
            },
            responsiveAnimationDuration: 0,
            plugins: {
                streaming: {
                    frameRate: 30,
                    refresh: 500,
                    delay: 500,
                    duration: 10000,
                    onRefresh: function (chart) {
                        if (depth != null) {
                            chart.data.datasets[0].data.push({ x: Date.now(), y: depth.depth });
                        }
                        if (dvl != null) {
                            chart.data.datasets[1].data.push({ x: Date.now(), y: dvl.altitude });
                        }
                    }
                }
            }
        }
    });
}

function update_sensor_rate() {
    s1 = new Date().getTime() / 1000;
    if (s1 - imu_s0 > 1.0) {
        imu = null;
    }
    var rate;
    if (imu != null && imu_count > 0) {
        var t1 = to_sec(imu);
        rate = (imu_count / (t1 - imu_t0)).toFixed(1);
    } else {
        rate = 0;
    }
    var $table = $('#table');
    $table.bootstrapTable('updateRow', {
        index: 0,
        row: {
            rate: rate
        }
    });
    imu_count = 0;

    if (s1 - dvl_s0 > 1.0) {
        dvl = null;
    }
    if (dvl != null && dvl_count > 0) {
        var t1 = to_sec(dvl);
        rate = (dvl_count / (t1 - dvl_t0)).toFixed(1);
    } else {
        rate = 0;
    }
    var $table = $('#table');
    $table.bootstrapTable('updateRow', {
        index: 1,
        row: {
            rate: rate
        }
    });
    dvl_count = 0;

    if (s1 - depth_s0 > 1.0) {
        depth = null;
    }
    if (depth != null && depth_count > 0) {
        var t1 = to_sec(depth);
        rate = (depth_count / (t1 - depth_t0)).toFixed(1);
    } else {
        rate = 0;
    }
    var $table = $('#table');
    $table.bootstrapTable('updateRow', {
        index: 2,
        row: {
            rate: rate
        }
    });
    depth_count = 0;
}

function sensor_rate_row_style(row, index) {
    if (index == 0) {
        if (row.rate < 190) {
            return { classes: "danger" }
        } else if (row.rate < 195) {
            return { classes: "warning" }
        } else {
            return { classes: "success" }
        }
    } else if (index == 1) {
        if (row.rate < 3) {
            return { classes: "danger" }
        } else if (row.rate < 4) {
            return { classes: "warning" }
        } else {
            return { classes: "success" }
        }
    } else if (index == 2) {
        if (row.rate < 3) {
            return { classes: "danger" }
        } else if (row.rate < 4) {
            return { classes: "warning" }
        } else {
            return { classes: "success" }
        }
    }
    return {}
}

function init_camera_state() {
    $('#camera_btn').on('click', function (e) {
        if ($('#camera_btn').hasClass('btn btn-dark')) {
            $('#camera_btn').removeClass('btn btn-dark');
            $('#camera_btn').toggleClass('btn btn-success');
            $('#camera').attr("src", "http://localhost:8080/stream?topic=/camera/image&bitrate=20000");
            $('#camera').attr("data-src", null);
        } else {
            $('#camera_btn').removeClass('btn btn-success');
            $('#camera_btn').toggleClass('btn btn-dark');
            $('#camera').attr("src", null);
            $('#camera').attr("data-src", "holder.js/100%x300/white/text:Camera");
            Holder.run({ images: '#camera' });
        }
    });
}

function init_sonar_state() {
    $('#sonar_btn').on('click', function (e) {
        if ($('#sonar_btn').hasClass('btn btn-dark')) {
            $('#sonar_btn').removeClass('btn btn-dark');
            $('#sonar_btn').toggleClass('btn btn-success');
            $('#sonar').attr("src", "http://localhost:8080/stream?topic=/sonar_oculus_node/image&bitrate=20000");
            $('#sonar').attr("data-src", null);
        } else {
            $('#sonar_btn').removeClass('btn btn-success');
            $('#sonar_btn').toggleClass('btn btn-dark');
            $('#sonar').attr("data-src", "holder.js/100%x400/white/text:Sonar Oculus");
            Holder.run({ images: '#sonar' });
        }
    });
}

function to_sec(msg) {
    return msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9;
}

function degrees(rad) {
    return rad / Math.PI * 180.0;
}

function quaternion_to_euler_angles(q) {
    sinr = +2.0 * (q.w * q.x + q.y * q.z);
    cosr = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    var roll = degrees(Math.atan2(sinr, cosr));

    sinp = +2.0 * (q.w * q.y - q.z * q.x);
    if (Math.abs(sinp) >= 1)
        var pitch = Math.copysign(Math.PI / 2.0, sinp);
    else
        var pitch = Math.asin(sinp);
    pitch = degrees(pitch);

    siny = +2.0 * (q.w * q.z + q.x * q.y);
    cosy = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    var yaw = degrees(Math.atan2(siny, cosy));
    return [roll, pitch, yaw]
}

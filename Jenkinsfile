pipeline {
  agent any
  stages {

    stage('Build') {
      environment {
        HOME = "${WORKSPACE}"
      }

      agent {
        docker {
          image 'px4io/px4-docs:2018-06-14'
        }
      }

      steps {
        sh 'export'
        sh 'gitbook install'
        sh 'gitbook build'
        stash includes: '_book/', name: 'gitbook'
        // publish html
        publishHTML target: [
          reportTitles: 'PX4 Dev Guide',
          allowMissing: false,
          alwaysLinkToLastBuild: true,
          keepAll: true,
          reportDir: '_book',
          reportFiles: '*',
          reportName: 'PX4 Dev Guide'
        ]
      }

    } // Build

    stage('Deploy') {
      environment {
        GIT_COMMITTER_EMAIL = "bot@pixhawk.org"
        GIT_COMMITTER_NAME = "PX4BuildBot"
      }

      agent {
        docker {
          image 'px4io/px4-docs:2018-06-14'
        }
      }

      steps {
        sh 'export'
        unstash 'gitbook'
        withCredentials([usernamePassword(credentialsId: 'px4buildbot_github', passwordVariable: 'GIT_PASS', usernameVariable: 'GIT_USER')]) {
          sh('git clone https://${GIT_USER}:${GIT_PASS}@github.com/PX4/dev.px4.io.git')
        }
        sh('rm -rf dev.px4.io/*')
        sh('cp -r _book/* dev.px4.io/')
        sh('cd dev.px4.io; git add .; git commit -a -m "gitbook build update `date`"')
        sh('cd dev.px4.io; git push origin master')
      }

      when {
        anyOf {
          branch 'master'
        }
      }

    } // Deploy
  } // stages

  options {
    buildDiscarder(logRotator(numToKeepStr: '10', artifactDaysToKeepStr: '30'))
    timeout(time: 60, unit: 'MINUTES')
  }

}


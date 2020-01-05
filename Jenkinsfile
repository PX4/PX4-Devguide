pipeline {
  agent {
    docker {
      image 'px4io/px4-docs:2020-01-05'
    }
  }
  stages {

    stage('Build') {
      environment {
        HOME = "${WORKSPACE}"
      }

      steps {
        sh('export')
        checkout(scm)
        sh('gitbook install')
        sh('gitbook build')
        stash(includes: '_book/', name: 'gitbook')
        // publish html
        publishHTML(target: [
          reportTitles: 'PX4 Dev Guide',
          allowMissing: false,
          alwaysLinkToLastBuild: true,
          keepAll: true,
          reportDir: '_book',
          reportFiles: '*',
          reportName: 'PX4 Dev Guide'
        ])
      }

    } // Build

    stage('Deploy') {
      environment {
        GIT_AUTHOR_EMAIL = "bot@px4.io"
        GIT_AUTHOR_NAME = "PX4BuildBot"
        GIT_COMMITTER_EMAIL = "bot@px4.io"
        GIT_COMMITTER_NAME = "PX4BuildBot"
      }

      steps {
        sh('export')
        unstash('gitbook')
        withCredentials([usernamePassword(credentialsId: 'px4buildbot_github_personal_token', passwordVariable: 'GIT_PASS', usernameVariable: 'GIT_USER')]) {
          sh('git clone https://${GIT_USER}:${GIT_PASS}@github.com/PX4/dev.px4.io.git')
          sh('rm -rf dev.px4.io/${BRANCH_NAME}')
          sh('mkdir -p dev.px4.io/${BRANCH_NAME}')
          sh('cp -r _book/* dev.px4.io/${BRANCH_NAME}/')
          sh('cd dev.px4.io; git add ${BRANCH_NAME}; git commit -a -m "gitbook build update `date`"')
          sh('cd dev.px4.io; git push origin master')
          
        }
      }
      post {
        always {
          sh('rm -rf dev.px4.io')
        }
      }
      when {
        anyOf {
          branch "master";
          branch "v1.*"
        }
      }

    } // Deploy
  } // stages

  options {
    buildDiscarder(logRotator(numToKeepStr: '10', artifactDaysToKeepStr: '30'))
    skipDefaultCheckout()
    timeout(time: 60, unit: 'MINUTES')
  }

}

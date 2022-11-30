<a name="readme-top"></a>

<!--
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
-->
[![LinkedIn][linkedin-shield]][linkedin-url]


<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/Benji-UW/COBOT-Transducer-Control-Code">
    <img src="Debugging Scripts/figures/fullscan_render_test_4.jpg" alt="Logo" width="120" height="80">
  </a>

<h3 align="center">STANLEy</h3>

  <p align="center">
    Stabilization and Transducer Alignment for Nearby Laser Elastography
    <br />
    <a href="https://github.com/Benji-UW/COBOT-Transducer-Control-Code"><strong>Explore the docs »</strong></a>
</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>


<!-- ABOUT THE PROJECT -->
## About The Project
![BAIL LAB logo](http://depts.washington.edu/wangast/images/u336-15-crop-u123.png?crc=4107162486)
<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!--

### Built With

* [![Next][Next.js]][Next-url]
* [![Python][Python]][Next-url]
* [![React][React.js]][React-url]
* [![Vue][Vue.js]][Vue-url]
* [![Angular][Angular.io]][Angular-url]
* [![Svelte][Svelte.dev]][Svelte-url]
* [![Laravel][Laravel.com]][Laravel-url]
* [![Bootstrap][Bootstrap.com]][Bootstrap-url]
* [![JQuery][JQuery.com]][JQuery-url]

<p align="right">(<a href="#readme-top">back to top</a>)</p>
-->


<!-- GETTING STARTED -->

## Getting Started

To get a local copy up and running on your UR cobot follow these simple example steps.

### Prerequisites
Make sure your computer has Python (version 3.7+) installed, and that you have the specific python packages outlined in `requirements.txt`. To install them all at once run
```sh
pip install -r requirements.txt
```
from the root of the repository.

Jupyternotebooks are also commonly used for data analysis and visualization, it is recommended you have that installed either standalone or as an extension on your prefered IDE.

Sensor input is processed in MATLAB or LabView, but so long as the optimization value is sent to the required socket on a running socket server, there's no real restrictions on what you use. To read about how to define sensor input effectively, read about it in `/Documentation/Control_System_Documentation.pdf`.

### Installation

1. Ensure your computer has Python version 3+
2. Clone the respository
3. Connect your UR3e to your computer via ethernet and assign it the static IP `192.168.0.5`
   1. Make sure the UR3e has internally assigned its eithernet port the static IP `192.168.0.10`
4. Make sure your sensor and signal processing software complies with the interface outlined in `/Documentation/Control_System_Documentation.pdf`, or make sure to enable the "headless" boolean in `TransducerHoming.py`
5. Make sure the programs stored in `/Code_for_the_robot` is installed on the UR3e, with `before_start.script` assigned to run before the script and `robot_program.script` assigned to run after. You will have to further configure your UR3e depending on the dimensions and location of your particular sensor.
6. First run `server.py` to enable the socket server that handles communication between the components. Then run the scripts on the UR3e, it should quickly connect to the socket server. Then run `Transducer_homing.py` to enable the robot and follow the on-screen instructions.

For full robot installation and setup procedure, read the quickstart guide in `/Documentation/Control_System_Documentation.pdf`.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- USAGE EXAMPLES -->
## Usage

This project is being run in the University of Washington department of bioengineering. Most of the documentation will be found in the pdf in the Documentation folder. There is large technical debt and presently the code-base is inscrutible for people who aren't working with it directly, so don't hesitate to reach out if you have questions on how to adapt this code to your project.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- CONTRIBUTING -->
## Contributing

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- LICENSE ## License

Distributed under the MIT License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

-->


<!-- CONTACT ## Contact

Your Name - [@DELETE_ME](https://twitter.com/DELETE_ME) - DELETE_ME@DELETE_ME_client.com

Project Link: [https://github.com/Benji-UW/COBOT-Transducer-Control-Code](https://github.com/Benji-UW/COBOT-Transducer-Control-Code)

<p align="right">(<a href="#readme-top">back to top</a>)</p>
-->



<!-- ACKNOWLEDGMENTS 
## Acknowledgments

* []()
* []()
* []()

<p align="right">(<a href="#readme-top">back to top</a>)</p>

-->

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/Benji-UW/COBOT-Transducer-Control-Code.svg?style=for-the-badge
[contributors-url]: https://github.com/Benji-UW/COBOT-Transducer-Control-Code/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/Benji-UW/COBOT-Transducer-Control-Code.svg?style=for-the-badge
[forks-url]: https://github.com/Benji-UW/COBOT-Transducer-Control-Code/network/members
[stars-shield]: https://img.shields.io/github/stars/Benji-UW/COBOT-Transducer-Control-Code.svg?style=for-the-badge
[stars-url]: https://github.com/Benji-UW/COBOT-Transducer-Control-Code/stargazers
[issues-shield]: https://img.shields.io/github/issues/Benji-UW/COBOT-Transducer-Control-Code.svg?style=for-the-badge
[issues-url]: https://github.com/Benji-UW/COBOT-Transducer-Control-Code/issues
[license-shield]: https://img.shields.io/github/license/Benji-UW/COBOT-Transducer-Control-Code.svg?style=for-the-badge
[license-url]: https://github.com/Benji-UW/COBOT-Transducer-Control-Code/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/benji-anderson
[lab-screenshot]: http://depts.washington.edu/wangast/images/u336-15-crop-u123.png?crc=4107162486
[Next.js]: https://img.shields.io/badge/next.js-000000?style=for-the-badge&logo=nextdotjs&logoColor=white
[Next-url]: https://nextjs.org/
[Python]: https://www.python.org/
[React.js]: https://img.shields.io/badge/React-20232A?style=for-the-badge&logo=react&logoColor=61DAFB
[React-url]: https://reactjs.org/
[Vue.js]: https://img.shields.io/badge/Vue.js-35495E?style=for-the-badge&logo=vuedotjs&logoColor=4FC08D
[Vue-url]: https://vuejs.org/
[Angular.io]: https://img.shields.io/badge/Angular-DD0031?style=for-the-badge&logo=angular&logoColor=white
[Angular-url]: https://angular.io/
[Svelte.dev]: https://img.shields.io/badge/Svelte-4A4A55?style=for-the-badge&logo=svelte&logoColor=FF3E00
[Svelte-url]: https://svelte.dev/
[Laravel.com]: https://img.shields.io/badge/Laravel-FF2D20?style=for-the-badge&logo=laravel&logoColor=white
[Laravel-url]: https://laravel.com
[Bootstrap.com]: https://img.shields.io/badge/Bootstrap-563D7C?style=for-the-badge&logo=bootstrap&logoColor=white
[Bootstrap-url]: https://getbootstrap.com
[JQuery.com]: https://img.shields.io/badge/jQuery-0769AD?style=for-the-badge&logo=jquery&logoColor=white
[JQuery-url]: https://jquery.com 
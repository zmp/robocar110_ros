# Contribution Guidelines

## General Guidelines
Code submission process is described in the [**official documentation**](https://docs.github.com/en/get-started/quickstart/contributing-to-projects).

This repository is under [**MIT License**](LICENSE), so the code you upload is going to have the same license. For more information, see the [**article**](https://docs.github.com/en/github/site-policy/github-terms-of-service#6-contributions-under-repository-license).

Please, don't upload content that infringes any proprietary right of any party, including patent, trademark, trade secret, copyright, right of publicity, or other right. More details [**here**](https://docs.github.com/en/github/site-policy/github-acceptable-use-policies).

## Versioning
* We are using [**semver.org**](https://semver.org/).
* RC110 nodes versions are the same as it's usually done in ROS.
* Each package.xml version is the same as CMakeLists.txt project version.

## Code Quality
* Please, document classes, public/protected methods, README, etc.
* Check that all dependencies are mentioned in `package.xml`.
* Follow existing naming conventions.
* Apply `.clang-format` file and test your code before commit.

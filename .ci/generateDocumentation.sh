#!/bin/sh
#
#    Title         : generateDocumentation.sh
#    Date created  : 2018
#    Notes         : original script from Jeroen de Bruijn
#    Author        : Diego Ferigo
#    Description   : This script is used to generated doxygen and mkdocs
#                    documentation and automatically deploy it to a gh-pages
#                    branch exploiting Travis CI
#
#    Preconditions:
#    --------------
#
#  - The script should be executed from the root of the repository, marked
#    by the PROJECT_DIR_ABS Travis' environment variable.
#  - Packages doxygen doxygen-doc doxygen-gui graphviz python-pip
#    must be installed.
#  - Pip packages Pygments mkdocs mkdocs-material must be installed.
#  - Doxygen configuration file must have the destination directory empty and
#    source code directory with a $(TRAVIS_BUILD_DIR) prefix.
#  - An gh-pages branch should already exist. See below for more info on how to
#    create a gh-pages branch.
#  - The deploy is performed by a bot github account that must have write access
#    to the repository. The authentication is performed with a deploy token with
#    repo permissions enabled. This token is stored in the Travis web interface as
#    private Environment Variable.
#  - Doxygen's Doxyfile stored in the doc/doxygen folder.
#  - MkDocs's mkdocs.yml stored in the doc/mkdocs folder.
#
#    Required Travis' global variables:
#    ---------------------------------
#
#    TRAVIS_BUILD_DIR       : The folder where the repository was cloned
#    TRAVIS_REPO_SLUG       : Slug of the repository (user/reponame)
#    TRAVIS_BUILD_NUMBER    : The number of the current build.
#    TRAVIS_COMMIT          : The commit that the current build is testing.
#
#    Required user-defined global variables:
#    ---------------------------------------
#
#    DEPLOY_TOKEN           : Secure token for authenticating the bot account.
#    GIT_COMMITTER_USERNAME : Github username of the bot
#    GIT_COMMITTER_NAME     : Name of the bot to set the git committer
#    GIT_COMMITTER_EMAIL    : Email of the bot to set the git committer
#    DOXYGEN_INPUT_FOLDER   : Folder containing Doxyfile
#    MKDOCS_INPUT_FOLDER    : Folder containing mkdocs.yml file
#
#    For information on how to create a clean gh-pages branch from the master
#    branch, please go to https://gist.github.com/vidavidorra/846a2fc7dd51f4fe56a0
#
#    This script will generate Doxygen and MkDocs documentation and push the files to
#    the gh-pages branch of the repository.
#    Before this script is used a gh-pages branch in the repository should aready exist.
#

# ================
# SETUP THE SCRIPT
# ================

# Exit with nonzero exit code if anything fails
set -e

echo 'Setting up the script...'

GH_REPO_ORG=${TRAVIS_REPO_SLUG%*/*}
GH_REPO_NAME=${TRAVIS_REPO_SLUG#*/*}
GH_REPO_REF="github.com/$GH_REPO_ORG/$GH_REPO_NAME.git"

# Folder where the gh-pages branch is cloned.
# This will be the root of the static website.
GH_PAGES_ROOTDIR=$TRAVIS_BUILD_DIR/code_docs

# Check if the gh-pages branch exists
cd $TRAVIS_BUILD_DIR
git ls-remote --heads --exit-code https://$GH_REPO_REF gh-pages
if [ $? -gt 0 ] ; then
    echo '' >&2
    echo 'gh-pages branch does not exist!' >&2
    exit 1
fi

# Get the current gh-pages branch
git clone -b gh-pages https://$GH_REPO_REF $GH_PAGES_ROOTDIR
cd $GH_PAGES_ROOTDIR
# Set the push default to simple i.e. push only the current branch
git config push.default simple

# Need to create a .nojekyll file to allow filenames starting with an underscore
# to be seen on the gh-pages site. Therefore creating an empty .nojekyll file.
# Presumably this is only needed when the SHORT_NAMES option in Doxygen is set
# to NO, which it is by default. So creating the file just in case.
touch $GH_PAGES_ROOTDIR/.nojekyll

# ==============================
# GENERATE DOXYGEN DOCUMENTATION
# ==============================

if [ -n "$DOXYGEN_INPUT_FOLDER" ] ; then
    echo 'Generating Doxygen code documentation...'
    cd $DOXYGEN_INPUT_FOLDER
    doxygen Doxyfile 2>&1 | tee $GH_PAGES_ROOTDIR/doxygen.log

    if [ -d $DOXYGEN_INPUT_FOLDER/html ] && [ -f $DOXYGEN_INPUT_FOLDER/html/index.html ] ; then
        echo "Doxygen generation successful..."
    else
        echo '' >&2
        echo 'Warning: Doxygen failed!' >&2
        echo 'Warning: No documentation (html) files have been found!' >&2
        echo 'Warning: Not going to push the documentation to GitHub!' >&2
        exit 1
    fi

    rm -r $GH_PAGES_ROOTDIR/doxygen || true
    mv $DOXYGEN_INPUT_FOLDER/html $GH_PAGES_ROOTDIR/doxygen
fi

# =============================
# GENERATE MKDOCS DOCUMENTATION
# =============================

if [ -n "$MKDOCS_INPUT_FOLDER" ] ; then
    echo 'Generating MkDocs code documentation...'
    cd $MKDOCS_INPUT_FOLDER
    mkdocs build -c -s -v --site-dir $GH_PAGES_ROOTDIR/mkdocs 2>&1 | tee $GH_PAGES_ROOTDIR/mkdocs.log

    if [ -d $GH_PAGES_ROOTDIR/mkdocs ] && [ -f $GH_PAGES_ROOTDIR/mkdocs/index.html ] ; then
        echo "MkDocs generation successful!"
    else
        echo '' >&2
        echo 'Warning: MkDocs failed!' >&2
        echo 'Warning: No documentation (html) files have been found!' >&2
        echo 'Warning: Not going to push the documentation to GitHub!' >&2
        exit 1
    fi
fi

# ========================
# DEPLOY THE DOCUMENTATION
# ========================

if [ -z "$DOXYGEN_INPUT_FOLDER" ] && [ -z "$MKDOCS_INPUT_FOLDER" ] ; then
    echo '' >&2
    echo 'Nothing to generate!' >&2
    exit 1
fi

# Add a new remote containing the tokenized bot login credentials
cd $GH_PAGES_ROOTDIR
git remote add origin-botlogin https://$GIT_COMMITTER_USERNAME:$DEPLOY_TOKEN@$GH_REPO_REF

# Add everything in this directory to the gh-pages branch.
# If you want to ignore resources, push to gh-pages a .gitignore file.
cd $GH_PAGES_ROOTDIR
git add --all

# The commit author will be the author of the last commit, and the committer
# will be the bot (this is already configured by the GIT_COMMITTER_* variables)
COMMIT_AUTHOR="$(git --no-pager show -s --format='%an <%ae>' $TRAVIS_COMMIT)"

# Commit the added files with a title and description containing the Travis CI
# build number and the GitHub commit reference that issued this build.
git commit -m "Automatic docs deployment Travis#${TRAVIS_BUILD_NUMBER}" \
           -m "Commit: https://github.com/$TRAVIS_REPO_SLUG/commit/$TRAVIS_COMMIT" \
           --author "$COMMIT_AUTHOR"

# Force push to the remote gh-pages branch.
echo 'Uploading documentation to the gh-pages branch...'
git push --force origin-botlogin gh-pages

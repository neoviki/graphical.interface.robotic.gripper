rm -rf package-lock.json
rm -rf node_modules
npm install .
npm install --save-dev electron-rebuild
./node_modules/.bin/electron-rebuild


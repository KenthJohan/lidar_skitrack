for f in ../txtpoints/*/*.txt; do
echo "Using file $f"
./detection "$f"
sleep 1
done
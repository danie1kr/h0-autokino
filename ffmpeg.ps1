# create directory structure
New-Item -ItemType Directory -Path ".\sd" -ErrorAction SilentlyContinue
New-Item -ItemType Directory -Path ".\sd\movies" -ErrorAction SilentlyContinue

# convert each .mp4 here to 480x320 20fps series of jpegs
Get-ChildItem *.mp4 | ForEach-Object {
    $movieName =  ( $_.Name -replace ".mp4", "" )
    $movieName =  ( $movieName -replace " ", "" )
    $movieOutputPath = '.\sd\movies\' + ( $movieName -replace ".mp4", "" )
    $movieOutput = $movieOutputPath + '\%04d.jpg'

    New-Item -ItemType Directory -Path $movieOutputPath -ErrorAction SilentlyContinue
    ffmpeg.exe -i $_.Name -s 480x320 -filter:v "fps=20" $movieOutput
}

# create movies.json with largestJpeg, movie paths and number of frames
Write-Output "{" > .\movies.json
$maxFileSize = 0 
Get-ChildItem ".\sd\movies" -Recurse -File | ForEach-Object {
    $Size = $_.Length
    $maxFileSize = (Get-Variable -name maxFileSize, Size | Sort Value | Select -Last 1).Value
}
Write-Output " ""maxFileSize"": $((Get-Variable -name maxFileSize).Value)," >> .\movies.json
Write-Output " ""movies"": [" >> .\movies.json
Get-ChildItem ".\sd\movies" -Recurse -Dir | ForEach-Object {
    Write-Output "{ ""name"": ""$($_)"", ""frameCount"": $((dir .\sd\movies\$_ | measure).Count)}," >> .\movies.json
}
Write-Output "]" >> .\movies.json
Write-Output "}" >> .\movies.json

# copy to sd and convert to ascii
gc -en utf8 .\movies.json | Out-File -en ascii .\sd\movies\movies.json
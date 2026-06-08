# Installing ffmpeg on Windows

ffmpeg is a free video tool that the drone camera code needs to decode the video stream.

---

## Step 1 — Download ffmpeg

1. Go to **https://ffmpeg.org/download.html**
2. Under "Get packages & executable files", click the **Windows** icon
3. Click **"Windows builds by gyan.dev"**
4. Under "release builds", download **`ffmpeg-release-essentials.zip`**

---

## Step 2 — Extract and place the folder

1. Once downloaded, **right-click** the zip file → **Extract All**
2. You'll get a folder with a long name like:
   ```
   ffmpeg-2026-06-04-git-c27a3b12e3-essentials_build
   ```
3. Move or copy that folder somewhere permanent — for example:
   ```
   C:\Users\YourName\Documents\ffmpeg\
   ```
   > Don't leave it in Downloads — if you clean out Downloads it'll break.

4. Note the full path to the **bin** folder inside it, for example:
   ```
   C:\Users\camde\Documents\ffmpeg\ffmpeg-2026-06-04-git-c27a3b12e3-essentials_build\bin
   ```

---

## Step 3 — Add ffmpeg to your PATH

1. Press the **Windows key** and search **"environment variables"**
2. Click **"Edit the system environment variables"**
3. Click **"Environment Variables..."** (bottom right of the window)
4. Under **"User variables"**, click **Path** → click **"Edit"**
5. Click **"New"** and paste your bin path from Step 2, e.g.:
   ```
   C:\Users\camde\Documents\ffmpeg\ffmpeg-2026-06-04-git-c27a3b12e3-essentials_build\bin
   ```
6. Click **OK** on all open windows to save

---

## Step 4 — Verify it works

1. **Close and reopen** your terminal (PATH changes don't apply to already-open terminals)
2. Run:
   ```
   ffmpeg -version
   ```
3. You should see something like:
   ```
   ffmpeg version 2026-06-04-git-... Copyright (c) 2000-2026 the FFmpeg developers
   ```

If you see that, you're done. ✅

---

## Troubleshooting

**"ffmpeg is not recognized"** after restarting the terminal:
- Double-check the path in Environment Variables — make sure it points all the way to the `\bin` folder, not just the ffmpeg root folder
- Make sure you clicked OK on all windows (changes don't save if you hit Cancel or close the window)
- Try restarting your PC entirely

**Not sure where your bin folder is?** Run this in PowerShell to find it:
```powershell
Get-ChildItem -Path "C:\Users\$env:USERNAME" -Recurse -Filter "ffmpeg.exe" -ErrorAction SilentlyContinue | Select-Object FullName
```
That will print the exact path — use everything up to (and including) `\bin` in your PATH.
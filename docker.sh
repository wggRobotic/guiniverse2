cmd=(env)

[[ " $@ " =~ " -sim " ]] && cmd+=(SIM_MODE=false)

cmd+=(docker compose up guiniverse2)

echo "${cmd[@]}"
"${cmd[@]}"

docker compose down